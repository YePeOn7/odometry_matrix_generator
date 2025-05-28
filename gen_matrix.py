import math
import numpy as np

ROBOT_RADIUS = 1.0
WHEEL_RADIUS = 0.6
HEADING_OFFSET = 0
N = 5

def gen_kinematic_matrix(N, heading_offset = 0):
    del_angle = 360/N
    M = np.zeros((N, 3))

    for i in range(N):
        M[i, 0] = -math.sin( ( del_angle * i + heading_offset ) * math.pi / 180 ) / WHEEL_RADIUS
        M[i, 1] = math.cos( ( del_angle * i + heading_offset ) * math.pi / 180 ) / WHEEL_RADIUS
        M[i, 2] = ROBOT_RADIUS / WHEEL_RADIUS

    return M

def get_odometry_matrix(M):
    return np.linalg.pinv(M)[:2, :]

def format_matrix_as_c_array(matrix, var_name="M_c_array", dtype="float"):
    rows, cols = matrix.shape
    c_array_str = f"{dtype} {var_name}[{rows}][{cols}] = {{\n"
    for r in range(rows):
        row_values = []
        for c in range(cols):
            # Format each number with .Xf to ensure float representation in C
            row_values.append(f"{matrix[r, c]:.6f}f") # .6f for 6 decimal places, 'f' suffix for float literal
        c_array_str += "    {" + ", ".join(row_values) + "}"
        if r < rows - 1:
            c_array_str += ","
        c_array_str += "\n"
    c_array_str += "};"
    return c_array_str

M = gen_kinematic_matrix(N, HEADING_OFFSET)
MI = get_odometry_matrix(M)

# print(M)
# print(MI)

matrix_kinematic = format_matrix_as_c_array(M, "m_kinematic")
matrix_odometry = format_matrix_as_c_array(MI, "m_odom#etry")

print("// ---------- Matrix Kinematic ---------- //")
print(matrix_kinematic)
print("// ---------- Matrix Odometry ---------- //")
print(matrix_odometry)