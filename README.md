# pointcloudToMesh
C++ application to convert pcd file, ply file, txt file or xyz point cloud to MESH representation (Gp3). 

-------------------
## Input file structure support

* .pcd 
* .ply
* .txt
* .xyz

## Output file structure 
Mesh cloud:
* .ply

## Example
<img src="./example/mss1.png" align="center" height="400" width="700"><br>

## Compilation
* Set "YOUR OWN" PCL Build DIR in CMakeList.txt e.g: **/opt/pcl-1.8.1/build** and save it
* Create a "build" folder

in the main folder:

    - cd build  
    - cmake ../src/
    - make
       
        	 
### Test

    ./pointcloudToMESH <input cloud> <surface method estimation> <normal method estimation> <output dir>
    
    surface method estimation:
        1 --> for poisson
        2 --> for gp3
        
    normal method estimation.
        1 --> for normal estimation
        2 --> for mls normal estimation
        
     example:
     ./pointcloudToMESH /home/xXx/PCD-PLY_Files/cloud.txt 2 1 /home/xXx/Desktop    

  
You can modify the parameters in "create_mesh" method to get better results!
  
    for GP3:
    
    setKSearch(100);                         //It was 20
    gp3.setSearchRadius(10);                 //It was 0.025
    gp3.setMu(5);                            //It was 2.5
    gp3.setMaximumNearestNeighbors(100);     //It was 100
    gp3.setMaximumSurfaceAngle(M_PI/4);      //45 degrees   
    gp3.setMinimumAngle(M_PI/18);            //10 degrees 
    gp3.setMaximumAngle(M_PI/1.5);           //120 degrees     
    gp3.setNormalConsistency(false); 
    
    for POISSON:
    
    poisson.setDepth(depth);//9
    poisson.setInputCloud(cloud_with_normals);
    poisson.setPointWeight(pointWeight);//4
    poisson.setDegree(2);
    poisson.setSamplesPerNode(samplePNode);//1.5
    poisson.setScale(scale);//1.1
    poisson.setIsoDivide(isoDivide);//8
    poisson.setConfidence(confidence);
    poisson.setOutputPolygons(outputPolygons);
    poisson.setManifold(manifold);
    poisson.setSolverDivide(solverDivide);//8
    poisson.reconstruct(triangles);
