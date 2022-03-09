import numpy as np


def check_intersection(cur_p, goal_p, end_points):
    """
    Args:
        cur_p: np array
        goal_p: np array
        end_points: list of list [[p1x,p1y],[p2x,p2y]]
    Returns:
        is_intersect: True or Flase
    """
    # print("end points", end_points)
    if end_points == [] or len(end_points)==1:
        return False
    o1 = end_points[0]
    o2 = end_points[1]

    def orientation(p, q, r):
        val = ((q[1] - p[1]) * (r[0] - q[0])) - ((q[0] - p[0]) * (r[1] - q[1]))
        if val == 0:
            return 0
        return 1 if val > 0 else -1

    def on_segment(p, q, r):
        if r[0] <= max(p[0], q[0]) and r[0] >= min(p[0], q[0]) and r[1] <= max(p[1], q[1]) and r[1] >= min(p[1], q[1]):
            return True
        return False

    ori1 = orientation(o1, o2, cur_p)
    ori2 = orientation(o1, o2, goal_p)
    ori3 = orientation(cur_p, goal_p, o1)
    ori4 = orientation(cur_p, goal_p, o2)

    if ori1 != ori2 and ori3 != ori4:
        return True
    if ori1 == 0 and on_segment(o1, o2, cur_p): return True
    if ori2 == 0 and on_segment(o1, o2, goal_p): return True
    if ori3 == 0 and on_segment(cur_p, goal_p, o1): return True
    if ori4 == 0 and on_segment(cur_p, goal_p, o2): return True

    return False

cur_p=np.array([17.4, 22.8])
goal_point=np.array([21.5,28.5])
end_points=[[16,23],[19,23]]

result=check_intersection(cur_p, goal_point, end_points)
print(result)