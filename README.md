## icsfusion

A SLAM system, checkout to main branch. 

In this test we use CUDA 9.0

This is a test branch for isam.




## isam
Get/Install isam http://people.csail.mit.edu/kaess/isam/doc/

compile isam with add_compile_options(-fPIC) in isam_vX_Y/isamlib/CMakeLists.txt

make sure libisam.a is /usr/local/lib/


## BUILD 
mkdir build in ICS-Fusion/isamTest
mkdir build/f_
cd build && cmake ..
make
./isamTest
## OUTPUT FILES
f_X_graph = before optimization
f_X_graph = after optimization
f_X_poses = ground_truth