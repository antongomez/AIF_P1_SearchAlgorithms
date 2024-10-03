import numpy as np
matrix_sizes=[3,5,7,9]


def random_matrix(matrix_sizes, quantity):
    random=[]
    for size in matrix_sizes:
        rep = 0
        while rep < quantity:
            random_matrix.append(np.random.randint(0, 10, (size, size)))
            rep = rep

print(len(random), len(random[0], len(random[11], len(random[99]))))
#print(random_matrix[0][1][1])
