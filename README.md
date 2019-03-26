# pointcloudToMesh
C++ application to convert pcd file, ply file, txt file or xyz point cloud to MESH representation (POISSON RECONSTRUCTION and Gp3). 

-------------------
## Example
<img src="./example/mss.png" align="center" height="400" width="700"><br>

## Compile
* Set "YOUR OWN" PCL Build DIR in CMakeList.txt e.g: **/opt/pcl-1.8.1/build** and save it
* Create a "build" folder

in the main folder:

    - cd build  
    - cmake ../src/
    - make
       
        	 
### Test

    ./pointcloudToMESH <ply file> -o <output dir>
    ./pointcloudToMESH <pcd file> -o <output dir>
    ./pointcloudToMESH <txt file> -o <output dir>
    ./pointcloudToMESH <xyz file> -o <output dir>


