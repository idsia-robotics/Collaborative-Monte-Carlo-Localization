
#import numpy as np
#import pandas as pd
from goodpoints import compress

def compresspp_kt_test(X, g=3):

    kernel_id=0
    kernel_type = b""
    if kernel_id == 0:
        kernel_type = b"gaussian"
    elif kernel_id == 1:
         kernel_type = b"sobolev" 
    else:
        print("Invalid kernel_id")
        exit()

    #print(X.shape)

    #df = pd.DataFrame(X)
    #df.to_csv("pydump.csv", header=False, index=False)

    coreset = compress.compresspp_kt(X, b"gaussian", g=g)

    #X_sampled = X[coreset]
    #print(coreset)
    #print(X_sampled)
    return coreset










