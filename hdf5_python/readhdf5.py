import h5py
import numpy as np
import os
from plyfile import (PlyData, PlyElement, make2d, PlyParseError, PlyProperty)
import sys
'''
def load_ply_data(filename, point_num):
    plydata = PlyData.read(filename)
    pc = plydata['vertex'].data[:point_num]
    pc_array = np.array([[pc[i][0], pc[i][1], pc[i][2]] for i in range(len(pc))])
    return pc_array


def save_h5(h5_filename, data, data_dtype='float32'):
    h5_fout = h5py.File(h5_filename)
    h5_fout.create_dataset(
            'data', data=data,
            compression='gzip', compression_opts=4,
            dtype=data_dtype)
    h5_fout.close()


rootdir = sys.argv[1]
data =[]
filelist = os.listdir(rootdir)
for i in range(0,len(filelist)):
    path = os.path.join(rootdir,filelist[i])
    a = load_ply_data(path, 2048)
    
    #if i==0: 
    #    pre = a
    #else:
    #    pre = np.vstack((a,pre))
   
    data.append(a)
    print('加入点云数据:   '+path)
    print(a.shape) 

data = np.array(data)
print(data.shape)
save_h5('1.h5',data)
'''
f = h5py.File(sys.argv[1],'r')#'1.h5'
f.keys()
print(f.keys())
key = ""
for k in f.keys():
   key=k
   print(key)
   print(f[key][:])
   print(f[key].shape)




