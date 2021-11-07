#version 120
precision highp float;

#include "helpsettings.glsl"
varying vec3  screenCoordsPassthrough;
varying vec4  position;
varying float linearDepth;

#ifdef HEIGHT_FOG
varying vec3 fogH;
uniform mat4 osg_ViewMatrixInverse;
#endif

void main(void)
{
    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;

    mat4 scalemat = mat4(0.5, 0.0, 0.0, 0.0,
                         0.0, -0.5, 0.0, 0.0,
                         0.0, 0.0, 0.5, 0.0,
                         0.5, 0.5, 0.5, 1.0);

    vec4 texcoordProj = ((scalemat) * ( gl_Position));
    screenCoordsPassthrough = texcoordProj.xyw;

    position = gl_Vertex;

    linearDepth = gl_Position.z;

#ifdef HEIGHT_FOG
    fogH = (osg_ViewMatrixInverse * (gl_ModelViewMatrix * gl_Vertex)).xyz;
#endif
}
