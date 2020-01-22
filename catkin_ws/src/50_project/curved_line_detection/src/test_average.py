import numpy as np

points = [[[319, 314], [622, 314]],
          [[319, 301], [608, 301]],
          [[319, 287], [596, 287]],
          [[319, 274], [588, 274]],
          [[320, 260], [591, 260]],
          [[320, 247]]]
N = 1
points = [points[i * N:(i + 1) * N] for i in range((len(points) + N - 1) // N)]
points_tmp = []
for point_chunks in points:
    try:
        line = np.array(point_chunks)
        line_tmp = []
        for i in range(line.shape[1]):
            print line[:, i]
            point = list(np.average(line[:, i], axis=0).astype(int))
            print point
            line_tmp.append(point)
            print "---------------"

        points_tmp.append(line_tmp)

    except Exception:
        for line in point_chunks:
            points_tmp.append(line)

    print "##############"

print points_tmp
