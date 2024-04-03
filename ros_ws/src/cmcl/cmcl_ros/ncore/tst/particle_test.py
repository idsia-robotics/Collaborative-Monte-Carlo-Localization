from goodpoints import kt
#from util_k_mmd import kernel_eval
import numpy as np
# for partial functions, to use kernel_eval for kernel
#from functools import partial
from goodpoints.tictoc import tic, toc
import pandas as pd
import matplotlib.pyplot as plt


from goodpoints import compress

def compresspp_kt_test(X, kernel_id=0, g=0):

    kernel_type = b""
    if kernel_id == 0:
        kernel_type = b"gaussian"
    elif kernel_id == 1:
         kernel_type = b"sobolev" 
    else:
        print("Invalid kernel_id")
        exit()

    coreset = compress.compresspp_kt(X, kernel_type, g=g)

    X_sampled = X[coreset]
    print(coreset)
    print(coreset.shape)
    return X_sampled


# def kernel_thinning_test(X, m=8):

#     d = int(2)
#     var = 1.

#     params_k_swap = {"name": "gauss", "var": var, "d": int(d)}
#     params_k_split = {"name": "gauss_rt", "var": var/2., "d": int(d)}

#     split_kernel = partial(kernel_eval, params_k=params_k_split)
#     swap_kernel = partial(kernel_eval, params_k=params_k_swap)
#     coreset = kt.thin(X, m, split_kernel, swap_kernel, delta=0.5, seed=None, store_K=False, 
#                       meanK=None, unique=False, verbose=False)

#     X_sampled = X[coreset]
#     print(coreset)
#     print(coreset.shape)
#     return X_sampled


def plot(X, X_sampled):

    plt.scatter(X[:, 0], X[:, 1], c='g', marker='o', alpha=0.2)
    plt.scatter(X_sampled[:, 0], X_sampled[:, 1], c='r', marker='o', alpha=1.0)

    plt.show()



def main():

    df = pd.read_csv("/home/nickybones/Code/docker_ros2/ros_ws/src/cmcl/cmcl_ros/ncore/tst/data/10000/sym4/particlesA_0.csv", index_col=False, header=None)
    data = df.to_numpy()
    X = data[:,[0, 1]]
    verbose = True

    #tic()
    #X_sampled = kernel_thinning_test(X)
    #toc(print_elapsed=verbose)

    tic()
    X_sampled =compresspp_kt_test(X, kernel_id=0)
    #X_sampled =compresspp_kt_test(X_sampled)
    toc(print_elapsed=verbose)
    print(X_sampled)

    plot(X, X_sampled)

if __name__ == '__main__':
    main()






