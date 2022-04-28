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

## Docker image
There is a docker image for this project stored in docker hub, [here](https://hub.docker.com/r/danieltobon43/pointcloud-to-mesh). This image is compiled with [pcl-docker-1.9.1](https://hub.docker.com/r/danieltobon43/pcl-docker-1-9-1), Ubuntu 20.04 and the mesh project (`5.85GB`aprox.).

To use it you have to install [docker-engine](https://docs.docker.com/engine/install/) in your host machine:

Download the docker image

```
docker pull danieltobon43/pointcloud-to-mesh:1.0-ubuntu-20-04
```

Check downloaded image
```
docker images
```

Run a docker container
```
docker run --rm -it \
           --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
           --volume=/tmp/.docker.xauth:/tmp/.docker.xauth:rw \
           --env="XAUTHORITY=/tmp/.docker.xauth" \
           --env="DISPLAY" \
           --name="mesh" \
           --cap-add sys_ptrace \
           -p 127.0.0.1:2222:22 \
           --user=pcl \
           --volume=[PATH TO YOUR PCD FOLDER]:/home/pcl/project/pcd \
           -t danieltobon43/pointcloud-to-mesh:1.0-ubuntu-20-04 pcd/[YOUR PCD FILENAME] [surface method] [normal method] [output dir]
```

The previous command will run a docker container with the `danieltobon43/pointcloud-to-mesh:1.0-ubuntu-20-04`  image and will share a `.pcd` file from the host machine (`[PATH TO YOUR PCD FOLDER]`) to the pcd folder in the container.

More information about this docker image can be found in the docker hub repository.

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
    
    poisson.setDepth(depth);//7
    poisson.setInputCloud(cloud_with_normals);
    poisson.setPointWeight(pointWeight);//2
    poisson.setDegree(2);
    poisson.setSamplesPerNode(samplePNode);//1.5
    poisson.setScale(scale);//1.1
    poisson.setIsoDivide(isoDivide);//8
    poisson.setConfidence(confidence);
    poisson.setOutputPolygons(outputPolygons);
    poisson.setManifold(manifold);
    poisson.setSolverDivide(solverDivide);//8
    poisson.reconstruct(triangles);
