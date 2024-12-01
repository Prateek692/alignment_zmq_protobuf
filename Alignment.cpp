#pragma once
#include <iostream>
#define _SILENCE_EXPERIMENTAL_FILESYSTEM_DEPRECATION_WARNING
#include <experimental/filesystem>
#include <fstream>
#include<memory>
#include<opencv2/core.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include <random>
#include <zmq.hpp>
#include<thread>
#include<chrono>
#include"ProtoMessages/message.pb.h"
#include"ProtoMessages/new_message.pb.h"

void send_protobuf(const std::vector<std::vector<double>>& data, const std::string& laser_type, zmq::socket_t& socket) {
    // Create Protobuf message
    Coordinates msg;
    for (const auto& sublist : data) {
        auto* coordinate = msg.add_unit();
        for (const auto& val : sublist) {
            coordinate->add_values(val);
        }
    }

    // Set the laser type
    msg.set_laser_type(laser_type);

    // Serialize the message
    std::string serialized_data;
    msg.SerializeToString(&serialized_data);

    // Send serialized data via ZMQ
    zmq::message_t zmq_msg(serialized_data.size());
    memcpy(zmq_msg.data(), serialized_data.data(), serialized_data.size());
    socket.send(zmq_msg, zmq::send_flags::none);
    std::cout << "Protobuf message sent!" << std::endl;
}

std::pair<std::vector<std::vector<double>>, std::string> receive_protobuf(zmq::socket_t& socket) {
    // Receive the serialized Protobuf message
    zmq::message_t zmq_msg;
    socket.recv(zmq_msg, zmq::recv_flags::none);

    // Deserialize into Protobuf message
    Coordinates msg;
    msg.ParseFromArray(zmq_msg.data(), zmq_msg.size());

    // Convert to std::vector<std::vector<double>> and extract laser type
    std::vector<std::vector<double>> data;
    for (const auto& coordinate : msg.unit()) {
        std::vector<double> sublist(coordinate.values().begin(), coordinate.values().end());
        data.push_back(sublist);
    }
    std::string laser_type = msg.laser_type();

    std::cout << "Protobuf message received!" << std::endl;
    return { data, laser_type };
}

void send_vector(const std::vector<std::vector<double>>& data, zmq::socket_t& socket) {
    // Send the size of the vector
    int vector_size = data.size() * 2;
    zmq::message_t size_msg(sizeof(vector_size));
    std::memcpy(size_msg.data(),&vector_size , sizeof(vector_size));
    socket.send(size_msg, zmq::send_flags::sndmore);

    // Send the vector data
    zmq::message_t data_msg(vector_size * sizeof(double));

    std::vector<double>serialized_cords;
    for (auto& coord : data) {
        //std::cout << coord[0] << "    " << coord[1] << std::endl;
        serialized_cords.push_back(coord[0]);
        serialized_cords.push_back(coord[1]);
    }
    std::cout << serialized_cords.size() << std::endl;

    std::memcpy(data_msg.data(), serialized_cords.data(), serialized_cords.size() * sizeof(double));
    socket.send(data_msg, zmq::send_flags::none);
}

std::vector<std::vector<double>> file_to_vector(std::string& filename) {
    std::vector<std::vector<double>> res;
    std::string line;
    std::ifstream file(filename);
    // Skip the header line (if present)
    std::getline(file, line);
    // Read data line by line
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string x_str, y_str;

        // Read values separated by a comma
        if (std::getline(ss, x_str, ',') && std::getline(ss, y_str, ',')) {
            try {
                float x = std::stof(x_str);
                float y = std::stof(y_str);
                res.push_back({ x,y });

            }
            catch (const std::exception& e) {
                std::cerr << "Error parsing line: " << line << " (" << e.what() << ")" << std::endl;
            }
        }
    }
    file.close();
    return res;
}

std::vector<std::vector<double>> receive_vector(zmq::socket_t& socket) {
    // Receive the size of the outer vector
    zmq::message_t outer_size_msg;
    socket.recv(outer_size_msg, zmq::recv_flags::none);
    int outer_size;
    std::memcpy(&outer_size, outer_size_msg.data(), sizeof(int));

    // Receive the sizes of each subvector
    zmq::message_t size_msg;
    socket.recv(size_msg, zmq::recv_flags::none);
    int coord_size;
    std::memcpy(&coord_size, size_msg.data(), sizeof(int));

    // Receive the flattened data
    zmq::message_t flat_data_msg;
    socket.recv(flat_data_msg, zmq::recv_flags::none);
    int total_size = flat_data_msg.size() / sizeof(double);
    std::vector<double> flat_data(total_size);
    std::memcpy(flat_data.data(), flat_data_msg.data(), flat_data_msg.size());

    // Reconstruct the vector of vectors
    std::vector<std::vector<double>> result;
    for (int i = 0; i < total_size; i += 2) {
        result.push_back({ flat_data[i],flat_data[i + 1] });
    }

    std::cout << "Received vector of vectors successfully." << std::endl;
    return result;
}

const size_t ROW_SIZE = 2; // Fixed row size

// Serialize a vector of vector of doubles into bytes
std::vector<char> serialize_coordinates(const std::vector<std::vector<double>>& coordinates) {
    size_t total_size = coordinates.size() * ROW_SIZE * sizeof(double);
    std::vector<char> bytes(total_size);

    double* ptr = reinterpret_cast<double*>(bytes.data());
    for (const auto& vec : coordinates) {
        if (vec.size() != ROW_SIZE) {
            throw std::runtime_error("All rows must have exactly 2 elements.");
        }
        for (double val : vec) {
            *ptr++ = val;
        }
    }
    return bytes;
}

// Deserialize bytes into a vector of vector of doubles
std::vector<std::vector<double>> deserialize_coordinates(const std::vector<char>& data) {
    const double* ptr = reinterpret_cast<const double*>(data.data());
    size_t total_doubles = data.size() / sizeof(double);

    if (total_doubles % ROW_SIZE != 0) {
        throw std::runtime_error("Data size is not compatible with the fixed row size.");
    }

    std::vector<std::vector<double>> coordinates;
    for (size_t i = 0; i < total_doubles; i += ROW_SIZE) {
        coordinates.emplace_back(ptr + i, ptr + i + ROW_SIZE);
    }
    return coordinates;
}

// Send 2D coordinates using ZMQ
void send_coordinates(zmq::socket_t& socket, const std::vector<std::vector<double>>& coordinates, const std::string& laser_type) {
    CoordinatesBytes msg;

    auto serialized = serialize_coordinates(coordinates);
    msg.set_coordinates_data(serialized.data(), serialized.size());
    msg.set_laser_type(laser_type);

    std::string serialized_msg;
    msg.SerializeToString(&serialized_msg);

    zmq::message_t zmq_msg(serialized_msg.size());
    memcpy(zmq_msg.data(), serialized_msg.data(), serialized_msg.size());
    socket.send(zmq_msg, zmq::send_flags::none);
    std::cout << "Message sent!" << std::endl;
}

// Receive 2D coordinates using ZMQ
std::pair<std::vector<std::vector<double>>, std::string> receive_coordinates(zmq::socket_t& socket) {
    zmq::message_t zmq_msg;
    socket.recv(zmq_msg, zmq::recv_flags::none);

    CoordinatesBytes msg;
    msg.ParseFromArray(zmq_msg.data(), zmq_msg.size());

    auto coordinates = deserialize_coordinates(std::vector<char>(msg.coordinates_data().begin(), msg.coordinates_data().end()));
    std::string laser_type = msg.laser_type();
    return { coordinates, laser_type };
}


int main() {

	std::string filename_flange = "D:\\wps_testing_data\\Shiny_wheel_images\\Data\\Flange.csv";
	std::string filename_tread = "D:\\wps_testing_data\\Shiny_wheel_images\\Data\\Tread.csv";
    std::vector<std::vector<double>> flange = file_to_vector(filename_flange);
    std::vector<std::vector<double>> tread = file_to_vector(filename_tread);
	
    std::cout << "Flange Points:" << flange.size() << std::endl;
    std::cout << "Tread Points:" << tread.size() << std::endl;

    zmq::context_t context(1);
    // Client socket for sending
    zmq::socket_t sender(context, zmq::socket_type::push);
    sender.connect("tcp://localhost:5555");
    // Server socket for receiving
    zmq::socket_t receiver(context, zmq::socket_type::pull);
    receiver.bind("tcp://*:5556");


    //std::string laser_type = "Flange";
    //send_protobuf(flange, laser_type, sender);
    //std::this_thread::sleep_for(std::chrono::seconds(2)); // Simulate delay
    //


    //laser_type = "Tread";
    //send_protobuf(tread, laser_type, sender);
    //std::this_thread::sleep_for(std::chrono::seconds(2)); // Simulate delay




    //std::cout << "Waiting for processed data..." << std::endl;

    //auto recd_data = receive_protobuf(receiver);
    //auto received_data = recd_data.first;
    //auto received_laser_type = recd_data.second;

    //std::cout << "Processed Data:" << std::endl;
    //for (const auto& vec : received_data) {
    //    for (const auto& val : vec) {
    //        std::cout << val << " ";
    //    }
    //    std::cout << std::endl;
    //}
    //std::cout << "Received Laser Type: " << received_laser_type << std::endl;


    //recd_data = receive_protobuf(receiver);
    //received_data = recd_data.first;
    //received_laser_type = recd_data.second;

    //// Print received data
    //std::cout << "Processed Data:" << std::endl;
    //for (const auto& vec : received_data) {
    //    for (const auto& val : vec) {
    //        std::cout << val << " ";
    //    }
    //    std::cout << std::endl;
    //}
    //std::cout << "Received Laser Type: " << received_laser_type << std::endl;

    std::string laser_type = "Flange";
    send_coordinates(sender, flange, laser_type);
    std::this_thread::sleep_for(std::chrono::seconds(2)); // Simulate delay



    laser_type = "Tread";
    send_coordinates(sender, tread, laser_type);
    std::this_thread::sleep_for(std::chrono::seconds(2)); // Simulate delay


    std::cout << "Waiting for processed data..." << std::endl;

    auto recd_data = receive_coordinates(receiver);
    auto received_data = recd_data.first;
    auto received_laser_type = recd_data.second;

    std::cout << "Processed Data:" << std::endl;
    for (const auto& vec : received_data) {
        for (const auto& val : vec) {
            std::cout << val << " ";
        }
        std::cout << std::endl;
    }
    std::cout << "Received Laser Type: " << received_laser_type << std::endl;


    recd_data = receive_coordinates(receiver);
    received_data = recd_data.first;
    received_laser_type = recd_data.second;

    // Print received data
    std::cout << "Processed Data:" << std::endl;
    for (const auto& vec : received_data) {
        for (const auto& val : vec) {
            std::cout << val << " ";
        }
        std::cout << std::endl;
    }
    std::cout << "Received Laser Type: " << received_laser_type << std::endl;


	return 0;
}
