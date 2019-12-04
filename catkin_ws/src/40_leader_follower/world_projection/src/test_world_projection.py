import numpy as np
import world_projection

r_matrix = np.array([-0.0031492212495974376, -0.34023754131384776, 0.9403342479598564, -0.9999925768464749,
    -0.0010161312944974607, -0.003716682544870674, 0.002220057987550638, -0.9403389723700173,
    -0.34023181565607885]).reshape(3, 3)
t_matrix = np.array([0.05136152075211747, 0.0005203214221408542, 0.28160219512361456]).reshape(3, 1)
k_matrix = np.array([318.7217102050781, 0.0, 319.86431884765625, 0.0, 0.0, 318.7640380859375,
    237.71546936035156, 0.0, 0.0, 0.0, 1.0, 0.0]).reshape(3, 4)

print r_matrix
print t_matrix
print k_matrix

wp = world_projection.WorldProjector(
            None,
            r_matrix,
            t_matrix,
            k_matrix)

print wp.pixel2world([443, 659])