# PCL Project
The purpose of this project is to read a point cloud from a PCD file, then perform 
operations on this point cloud and save the final point cloud to a PCD. After a point 
cloud is read from the file, it is transmitted by reference throughout the program. 
Otherwise, it will make the program very slow, because the point cloud is too large. In 
the program, a point cloud is segmented and colored with both RANSAC and Region 
Growing algorithms. The coloring part is performed with a function belonging to the 
Segmentation class.

Outputs of the project;
![image](https://github.com/gfbpae/PCL_Project/assets/94529874/ebe53d0b-b15e-4c4a-9dfb-04248c90fb83)

UML Diagram of the project :
![image](https://github.com/gfbpae/PCL_Project/assets/94529874/b7fd2944-a802-4ba1-b7e4-91f58eb8d685)

