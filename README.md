# pointcloudToMesh
C++ application to convert pcd file, ply file, txt file or xyz point cloud to MESH representation (Gp3). 

-------------------
## Example
<img src="./example/mss1.png" align="center" height="400" width="700"><br>

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

You can modify the parameters in "CreateMesh" method at main.cpp to get better results!

 setKSearch(100);                         //It was 20
 gp3.setSearchRadius(10);                 //It was 0.025
 gp3.setMu(5);                            //It was 2.5
 gp3.setMaximumNearestNeighbors(100);     //It was 100
 gp3.setMaximumSurfaceAngle(M_PI/4);      //45 degrees   
 gp3.setMinimumAngle(M_PI/18);            // 10 degrees 
 gp3.setMaximumAngle(M_PI/1.5);           // 120 degrees     
 gp3.setNormalConsistency(false); 
