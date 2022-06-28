# Orbifold visualization

## Specification

Create a ray tracing program in C++ that displays a dodecahedron room in a sphere of radius √3 m. The room contains an optically smooth golden object defined by an implicit equation f(x,y,z)=exp(ax^2+by^2-cz)-1 cut into a sphere of radius 0.3 m in the center of the room and a point source of light. The walls of the room from the corner to 0.1 m are of diffuse-specular type, with portals opening onto another similar room, but rotated 72 degrees about the center of the wall and mirrored onto the plane of the wall. The light source does not shine through the portal, each room has its own light source. During the display, it is sufficient to step through the portals a maximum of 5 times. The virtual camera faces the centre of the room and rotates around it.

Refractive index and extinction coefficient of gold: n/k: 0.17/3.1, 0.35/2.7, 1.5/1.9

The other parameters can be chosen individually, so that the image is aesthetic. a,b,c are positive non-integer numbers.

## Results
![results](results.gif)



