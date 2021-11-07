#version 120

uniform mat4 projectionMatrix;

varying vec3  screenCoordsPassthrough;
varying vec4  position;
varying float linearDepth;
uniform float osg_SimulationTime;
uniform mat4 osg_ViewMatrixInverse;

#include "shadows_vertex.glsl"
#include "depth.glsl"

void main(void)
{
    vec4 glvertice = gl_Vertex;
	
	vec4 campos = osg_ViewMatrixInverse * vec4(0.0, 0.0, 0.0, 1.0);
	vec4 viewPos_alt = (gl_ModelViewMatrix * gl_Vertex);
	float euclideanDepth = length(viewPos_alt.xyz);

	float frequency = 2.0*3.1415/0.1;

    float phase = 0.02 * frequency;
    
	
	
	if(euclideanDepth < 600000.0) {
	
	glvertice.xy *= 0.03;
	
	float tloop = mod(osg_SimulationTime, 201.0);

	float theta = dot(vec2(0.6,0.4), vec2(glvertice.xy));
	float sinres = sin(theta * frequency + tloop * phase);
	float h = pow((sinres + 1.0) * 0.5, 2.5);
	
	float g = 70.0 * 9.8;
	glvertice.z -= 25.0 * sqrt(h);
	}
	
	if(campos.z < -1.0)
	glvertice.z += 25.0; 

    gl_Position = projectionMatrix * (gl_ModelViewMatrix * glvertice);

    mat4 scalemat = mat4(0.5, 0.0, 0.0, 0.0,
                         0.0, -0.5, 0.0, 0.0,
                         0.0, 0.0, 0.5, 0.0,
                         0.5, 0.5, 0.5, 1.0);

    vec4 texcoordProj = ((scalemat) * ( gl_Position));
    screenCoordsPassthrough = texcoordProj.xyw;

    position = glvertice;

    vec4 viewPos = gl_ModelViewMatrix * glvertice;
    linearDepth = getLinearDepth(gl_Position.z, viewPos.z);

    setupShadowCoords(viewPos, normalize((gl_NormalMatrix * gl_Normal).xyz));
}
