# pointcloudToMesh
C++ application to convert a pcd, ply, txt or xyz point cloud data file into a MESH representation (Gp3/poisson). This projects is based in the Point Cloud Library 1.12.1 as backend.

-------------------
## Input file structure support

| Format      | Description |
| ----------- | ----------- |
| .pcd        | Point Cloud Data file format       |
| .ply        | Polygon file format                   |
| .txt        | Text file format                      |
| .xyz        | X Y Z Text file format             |

## Output file structure 

| Mesh Cloud      | Description |
| ----------- | ----------- |
| .ply   | Polygon mesh file       |

## Example
<p align="center">
   <img src="./example/mss1.png" align="center" height="400" width="700"><br>
</p>

## Compile
You can either build the project from source or download a docker image.

### Compiling from source code
1. Download source code:
```
git clone https://github.com/danielTobon43/pointcloudToMesh
```

2. Create a "build" folder
3. Run CMake

```
cd build/ && cmake ../src && make
```
       
4. Run project
```
./pointcloudToMESH <input cloud> <surface method estimation> <normal method estimation> <output dir>
    
    surface method estimation:
        1 --> for poisson
        2 --> for gp3
        
    normal method estimation.
        1 --> for normal estimation
        2 --> for mls normal estimation
        
     example:
     ./pointcloudToMESH /home/xXx/PCD-PLY_Files/cloud.txt 2 1 /home/xXx/Desktop 
```

### Docker image
1. Download the docker image

```
docker pull ghcr.io/danieltobon43/pcdtomesh:latest
```

2. Run a docker container
```
docker run --rm -it \
           --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
           --volume=/tmp/.docker.xauth:/tmp/.docker.xauth:rw \
           --env="XAUTHORITY=/tmp/.docker.xauth" \
           --env="DISPLAY" \
           --name="mesh" \
           --volume=[PATH TO YOUR PCD FOLDER]:/tmp \
           -t ghcr.io/danieltobon43/pcdtomesh:latest /tmp/[YOUR PCD FILENAME] [surface method] [normal method] /tmp
```

The previous command will run a docker container with the `ghcr.io/danieltobon43/pcdtomesh:latest` image and will share a `.pcd` file from the host machine (`[PATH TO YOUR PCD FOLDER]`) to the tmp folder in the container.
  
## Note  
These parameters can be modify in "create_mesh" method to get better results!
  
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
