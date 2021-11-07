#version 120
precision highp float;
    
varying vec3  screenCoordsPassthrough;
varying vec4  position;
varying float linearDepth;
uniform float osg_SimulationTime;
uniform mat4 osg_ViewMatrixInverse;

#include "helpsettings.glsl"

void main(void)
{
	vec4 glvertice = gl_Vertex;
	
#ifndef underwaterFog

	vec4 campos = osg_ViewMatrixInverse * vec4(0.0, 0.0, 0.0, 1.0);
	vec4 viewPos = (gl_ModelViewMatrix * gl_Vertex);
	float euclideanDepth = length(viewPos.xyz);
	vec2 dir = normalize(viewPos.xy - glvertice.xy);
	
	float frequency = 2.0*3.1415/0.1;

    float phase = 0.02 * frequency;
    
	
	
	if(euclideanDepth < 600000.0) {
	
	glvertice.xy *= 0.03;

	float theta = dot(vec2(0.6,0.4), vec2(glvertice.xy));
	float sinres = sin(theta * frequency + osg_SimulationTime * phase);
	float h = pow((sinres + 1.0) * 0.5, 2.5);
	
	float g = 70.0 * 9.8;
	glvertice.z -= 10.0/sqrt(g) * sqrt(h * g);
	}
	
	if(campos.z < -1.0)
	glvertice.z += 25.0; 
#endif
	
    gl_Position = gl_ModelViewProjectionMatrix * glvertice;

    mat4 scalemat = mat4(0.5, 0.0, 0.0, 0.0,
                         0.0, -0.5, 0.0, 0.0,
                         0.0, 0.0, 0.5, 0.0,
                         0.5, 0.5, 0.5, 1.0);

    vec4 texcoordProj = ((scalemat) * ( gl_Position));
    screenCoordsPassthrough = texcoordProj.xyw -
	vec3(0.0,0.0,0.0);

    position = glvertice;

    linearDepth = gl_Position.z;

}
