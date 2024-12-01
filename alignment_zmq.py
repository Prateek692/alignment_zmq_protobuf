import zmq
import numpy as np
import struct
from ProtoMessages import message_pb2,new_message_pb2
from alignment_code import align_lasers


def send_protobuf_test(data, laser_type, socket):
    """Serialize Protobuf and send via ZMQ."""
    msg = message_pb2.Coordinates()

    # Populate the Protobuf message
    for sublist in data:
        coordinate = msg.unit.add()
        coordinate.values.extend(sublist)

    # Set the laser type
    msg.Laser_type = laser_type

    # Serialize the message to bytes
    serialized_data = msg.SerializeToString()

    # Send the serialized data
    socket.send(serialized_data)
    print("Protobuf message sent!")

def receive_protobuf_test(socket):
    """Receive and deserialize Protobuf message from ZMQ."""
    serialized_data = socket.recv()

    # Deserialize the data into a Protobuf message
    msg = message_pb2.Coordinates()
    msg.ParseFromString(serialized_data)

    # Convert to Python data structures
    data = [[val for val in coordinate.values] for coordinate in msg.unit]
    laser_type = msg.Laser_type
    print("Protobuf message received!")
    return data, laser_type

def serialize_coordinates(coordinates):
    """Serialize a list of 2D coordinates to bytes."""
    flat_data = [value for pair in coordinates for value in pair]  # Flatten 2D list
    return struct.pack(f"{len(flat_data)}d", *flat_data)


def deserialize_coordinates(data_bytes):
    """Deserialize bytes to a list of 2D coordinates."""
    num_doubles = len(data_bytes) // 8  # Each double is 8 bytes
    flat_data = struct.unpack(f"{num_doubles}d", data_bytes)
    return [flat_data[i:i + 2] for i in range(0, len(flat_data), 2)]


def send_coordinates(socket, coordinates, laser_type):
    """Send serialized 2D coordinates using ZMQ."""
    msg = new_message_pb2.CoordinatesBytes()
    msg.coordinates_data = serialize_coordinates(coordinates)
    msg.Laser_type = laser_type
    socket.send(msg.SerializeToString())
    print("Message sent!")


def receive_coordinates(socket):
    """Receive serialized 2D coordinates using ZMQ."""
    msg = new_message_pb2.CoordinatesBytes()
    msg.ParseFromString(socket.recv())
    coordinates = deserialize_coordinates(msg.coordinates_data)
    return coordinates, msg.Laser_type


def send_vector_of_vectors(data, socket):
    """Send a list of lists via ZeroMQ."""
    # Flatten the list of lists
    flat_data = [item for sublist in data for item in sublist]
    coord_size = 2 
    list_size = len(data)

    # Serialize data
    outer_size_msg = struct.pack("i", list_size)  # Outer size as int
    size_msg = struct.pack("i", coord_size)  # Size of each sublist(coordinate)
    flat_data_msg = np.array(flat_data, dtype=np.float64).tobytes()  # Flattened array

    # Send messages
    socket.send(outer_size_msg, zmq.SNDMORE)
    socket.send(size_msg, zmq.SNDMORE)
    socket.send(flat_data_msg)
    print(struct.unpack("i", outer_size_msg)[0],struct.unpack("i", size_msg)[0])
    print("Sent vector of vectors successfully.")

def receive_vector_of_vectors(socket):
    """Receive a list of lists via ZeroMQ."""
    # Receive the size of the coordinates vector
    size_msg = socket.recv()
    vector_size = struct.unpack("i", size_msg)[0]  # Vector size would be integer(i)
    # Receive the flattened data
    data_msg = socket.recv()
    flat_data = np.frombuffer(data_msg, dtype=np.float64, count=vector_size)
    flat_data = flat_data.tolist()
    # print(flat_data)
    # Reconstruct the list of lists
    coordinates = []
    for i in range(0,vector_size,2):
        coordinates.append([flat_data[i],flat_data[i+1]])

    print("Received vector of coordinates successfully.")
    return coordinates


if __name__ == "__main__":
    # Setup ZeroMQ socket for both sending and receiving
    context = zmq.Context()

    # Server socket for receiving
    receiver = context.socket(zmq.PULL)
    receiver.bind("tcp://*:5555")

    # Client socket for sending
    sender = context.socket(zmq.PUSH)
    sender.connect("tcp://localhost:5556")

    # Laser Alignment class object
    alignment = align_lasers()

    # while True:
    #     print("Waiting for data...")
    #     received_data1, laser_type1 = receive_protobuf(receiver)
    #     print("Received Data Laser Type:", laser_type1)
    #     received_data2, laser_type2 = receive_protobuf(receiver)
    #     print("Received Data Laser Type:", laser_type2)


    #     # Process and send back
    #     flange = []
    #     tread = []
    #     if laser_type1 == "Flange" and laser_type2 == "Tread":
    #         flange,tread = alignment.do_alignment(received_data1,received_data2)
    #         send_protobuf(flange, laser_type1, sender)
    #         send_protobuf(tread, laser_type2, sender)
    #         print("Sending processed data back")
    #     elif laser_type1 == "Tread" and laser_type2 == "Flange":  
    #         flange,tread = alignment.do_alignment(received_data2,received_data1)
    #         send_protobuf(flange, laser_type2, sender)
    #         send_protobuf(tread, laser_type1, sender)
    #         print("Sending processed data back")
    #     else:
    #         print('Coordinates not received properly.')
    while True:
        print("Waiting for data...")
        received_coords1, received_laser_type1 = receive_coordinates(receiver)
        print("Received Data Laser Type:", received_laser_type1)
        received_coords2, received_laser_type2 = receive_coordinates(receiver)
        print("Received Data Laser Type:", received_laser_type2)
        
        # Process and send back
        flange = []
        tread = []
        if received_laser_type1 == "Flange" and received_laser_type2 == "Tread":
            flange,tread = alignment.do_alignment(received_coords1,received_coords2)
            send_coordinates(sender, flange, received_laser_type1)
            send_coordinates(sender, tread, received_laser_type2)
            print("Sending processed data back")
        elif received_laser_type1 == "Tread" and received_laser_type2 == "Flange":  
            flange,tread = alignment.do_alignment(received_coords2,received_coords1)
            send_coordinates(sender, flange, received_laser_type2)
            send_coordinates(sender, tread, received_laser_type1)
            print("Sending processed data back")
        else:
            print('Coordinates not received properly.')
