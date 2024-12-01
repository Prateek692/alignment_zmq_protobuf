import math
import numpy as np
from sklearn.linear_model import RANSACRegressor
from sklearn.neighbors import NearestNeighbors

class align_lasers:
    def __init__(self, max_iterations=1000, distance_threshold=1.5, convergence_translation_threshold=1e-3,
                 convergence_rotation_threshold=1e-5, point_pairs_threshold=10):
        """
        :param max_iterations: the maximum number of iteration to be executed
        :param distance_threshold: the distance threshold between two points in order to be considered as a pair
        :param convergence_translation_threshold: the threshold for the translation parameters (x and y) for the
                                                  transformation to be considered converged
        :param convergence_rotation_threshold: the threshold for the rotation angle (in rad) for the transformation
                                                   to be considered converged
        :param point_pairs_threshold: the minimum number of point pairs the should exist
        """
        self.max_iterations = max_iterations
        self.distance_threshold = distance_threshold
        self.convergence_translation_threshold = convergence_translation_threshold
        self.convergence_rotation_threshold = convergence_rotation_threshold
        self.point_pairs_threshold = point_pairs_threshold

    def calculate_rotation_angle(self,coords1, coords2):
    
        def fit_line_ransac(coordinates):
            X = coordinates[:, 0].reshape(-1, 1)  # Independent variable
            y = coordinates[:, 1]  # Dependent variable
            ransac = RANSACRegressor()
            ransac.fit(X, y)
            return ransac.estimator_.coef_[0]  # Return slope

        # Fit the lines and calculate slopes
        slope1 = fit_line_ransac(np.array(coords1))
        slope2 = fit_line_ransac(np.array(coords2))

        # Calculate the angle between the two slopes
        angle_radians = math.atan(abs((slope2 - slope1) / (1 + slope1 * slope2)))
        angle_degrees = math.degrees(angle_radians)
        return -angle_degrees
    
    def flange_corrected_rotation(self,flange,tread):
        shortlist_flange = []
        shortlist_tread = []
        x_min = float('inf')
        x_max = -float('inf')
        for point in flange:
            x_min = min(x_min,point[0])
            x_max = max(x_max,point[0])

        x_min = x_min + ((x_max-x_min) * 0.6)
        x_max = x_min + ((x_max-x_min) * 1)
        for point in flange:
            if point[0] > x_min and point[0] < x_max:
                shortlist_flange.append(point)

        x_min = float('inf')
        x_max = -float('inf')
        for point in tread:
            x_min = min(x_min,point[0])
            x_max = max(x_max,point[0])

        x_min = x_min + ((x_max-x_min) * 0.6)
        x_max = x_min + ((x_max-x_min) * 0.9)
        for point in tread:
            if point[0] > x_min and point[0] < x_max:
                shortlist_tread.append(point)

        closest_rot_angle = self.calculate_rotation_angle(shortlist_flange,shortlist_tread)
        theta = np.radians(closest_rot_angle)
        translation = np.array([0,0])
        rotation_matrix = np.array([
            [np.cos(theta), -np.sin(theta)],
            [np.sin(theta), np.cos(theta)]
        ])
        flange = [
            np.dot(rotation_matrix, coord) + translation
            for coord in flange
        ]
        return flange
    
    def point_based_matching(self,point_pairs):
        """
        This function is based on the paper "Robot Pose Estimation in Unknown Environments by Matching 2D Range Scans"
        by F. Lu and E. Milios.

        :param point_pairs: the matched point pairs [((x1, y1), (x1', y1')), ..., ((xi, yi), (xi', yi')), ...]
        :return: the rotation angle and the 2D translation (x, y) to be applied for matching the given pairs of points
        """

        x_mean = 0
        y_mean = 0
        xp_mean = 0
        yp_mean = 0
        n = len(point_pairs)

        if n == 0:
            return None, None, None

        for pair in point_pairs:
            (x, y), (xp, yp) = pair

            x_mean += x
            y_mean += y
            xp_mean += xp
            yp_mean += yp

        x_mean /= n
        y_mean /= n
        xp_mean /= n
        yp_mean /= n

        s_x_xp = 0
        s_y_yp = 0
        s_x_yp = 0
        s_y_xp = 0
        for pair in point_pairs:
            (x, y), (xp, yp) = pair

            s_x_xp += (x - x_mean) * (xp - xp_mean)
            s_y_yp += (y - y_mean) * (yp - yp_mean)
            s_x_yp += (x - x_mean) * (yp - yp_mean)
            s_y_xp += (y - y_mean) * (xp - xp_mean)

        rot_angle = math.atan2(s_x_yp - s_y_xp, s_x_xp + s_y_yp)
        translation_x = xp_mean - (x_mean * math.cos(rot_angle) - y_mean * math.sin(rot_angle))
        translation_y = yp_mean - (x_mean * math.sin(rot_angle) + y_mean * math.cos(rot_angle))

        return rot_angle, translation_x, translation_y
    
    def do_alignment(self,flange,tread):

        flange = np.array(flange)
        tread = np.array(tread)

        nbrs = NearestNeighbors(n_neighbors=1, algorithm='auto').fit(tread)
        for i in range(1000):
            closest_point_pairs = []  # list of point correspondences for closest point rule

            distances, indices = nbrs.kneighbors(flange)
            for nn_index in range(len(distances)):
                if abs(distances[nn_index][0]) < abs(self.distance_threshold):
                    closest_point_pairs.append((flange[nn_index], tread[indices[nn_index][0]]))

            if len(closest_point_pairs) < self.point_pairs_threshold:
                # if verbose:
                print(f'No better solution can be found (very few point pairs) at iteration {i}!')
                break

            closest_rot_angle, closest_translation_x, closest_translation_y = self.point_based_matching(closest_point_pairs)
            
            if closest_rot_angle is None or closest_translation_x is None or closest_translation_y is None:
                print('No better solution can be found!')
                break

            c, s = math.cos(closest_rot_angle), math.sin(closest_rot_angle)
            rot = np.array([[c, -s],
                            [s, c]])
            aligned_points = np.dot(flange, rot.T)
            aligned_points[:, 0] += closest_translation_x
            aligned_points[:, 1] += closest_translation_y

            flange = aligned_points

            if closest_rot_angle < 1e-5 and (max(abs(closest_translation_x),abs(closest_translation_y))<1e-3):
                print('Stopped at iteration:',i)
                break

                
        return flange.tolist(),tread.tolist()
        
    
# if __name__ == '__main__':
#     alignment = align_lasers()
#     flange = []
#     tread = []
#     flange,tread = alignment.do_alignment(flange,tread)
    
#     # for i in x:
#     #     print(i)
