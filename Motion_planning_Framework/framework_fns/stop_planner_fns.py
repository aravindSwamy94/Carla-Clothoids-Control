import numpy as np


def line_intersection(line1, line2):
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)

    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div
    return [x, y]


def pointOnSegment(p1, p2, p3):
    if (p2[0] <= max(p1[0], p3[0]) and (p2[0] >= min(p1[0], p3[0])) and \
       (p2[1] <= max(p1[1], p3[1])) and (p2[1] >= min(p1[1], p3[1]))):
        return True
    else:
        return False



def check_for_stop_source(waypoints, closest_index, goal_index, stop_line):

    for i in range(closest_index, goal_index):
        # Check to see if path segment crosses any of the stop lines.
        intersect_flag = False
        for key,stopsign_fence in enumerate(stop_line):
#            if self._stopsign_visited[key]: continue
            wp_1   = np.array(waypoints[i][0:2])
            wp_2   = np.array(waypoints[i+1][0:2])
            s_1    = np.array(stopsign_fence[0:2])
            s_2    = np.array(stopsign_fence[2:4])
            
            v1     = np.subtract(wp_2, wp_1)
            v2     = np.subtract(s_1, wp_2)
            sign_1 = np.sign(np.cross(v1, v2))
            v2     = np.subtract(s_2, wp_2)
            sign_2 = np.sign(np.cross(v1, v2))
            v1     = np.subtract(s_2, s_1)
            v2     = np.subtract(wp_1, s_2)
            sign_3 = np.sign(np.cross(v1, v2))
            v2     = np.subtract(wp_2, s_2)
            sign_4 = np.sign(np.cross(v1, v2))
            # Check if the line segments intersect.
            if (sign_1 != sign_2) and (sign_3 != sign_4):
                intersect_flag = True

            # Check if the collinearity cases hold.
            if (sign_1 == 0) and pointOnSegment(wp_1, s_1, wp_2):
                intersect_flag = True
            if (sign_2 == 0) and pointOnSegment(wp_1, s_2, wp_2):
                intersect_flag = True
            if (sign_3 == 0) and pointOnSegment(s_1, wp_1, s_2):
                intersect_flag = True
            if (sign_3 == 0) and pointOnSegment(s_1, wp_2, s_2):
                intersect_flag = True

            if intersect_flag:
                goal_index = i
                line1 = [[waypoints[i][0],waypoints[i][1]],
                         [waypoints[i+1][0],waypoints[i+1][1]]]
                line2 = [[stopsign_fence[0],stopsign_fence[1]],
                         [stopsign_fence[2],stopsign_fence[3]]]
                intersect_point = line_intersection(line1, line2)
#                self._stopsign_visited[key] = True
                return goal_index, True, intersect_point
    intersect_point=[]
    return goal_index, False,intersect_point

