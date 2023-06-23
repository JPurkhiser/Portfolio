import numpy as np
def integrateGraph(time, array):
    resArray = [0]
    for n in range(0, len(time)-1):
        resArray.append(resArray[-1] + 0.5*(array[n+1] + array[n])*(time[n+1] - time[n]))
        
    return np.array(resArray)