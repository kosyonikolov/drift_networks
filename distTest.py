import numpy as np

p0 = np.array([1, 1])
p1 = np.array([4, 10])
q  = np.array([3, 0])

mv  = p1 - p0
mv_norm = np.linalg.norm(mv)
mv  = mv / mv_norm
mvq = q - p0

a = np.dot(mvq, mv)
print(a)
print(a / mv_norm)


r = p0 + a * mv
print(r)

vr = r - q

check = np.dot(vr, mv)

print(check)
