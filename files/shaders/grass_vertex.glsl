#version 120

#if @diffuseMap
varying vec2 diffuseMapUV;
#endif

#define PER_PIXEL_LIGHTING @forcePPL

varying float euclideanDepth;

#if !@radialFog
varying float linearDepth;
#endif

#if @PER_PIXEL_LIGHTING
centroid varying vec4 passColor;
varying vec3 passViewPos;
varying vec3 passNormal;
#else
uniform int colorMode;
centroid varying vec4 lighting;
#include "lighting.glsl"
#endif

#if @grassAnimation
uniform float osg_SimulationTime;
uniform mat4 osg_ViewMatrixInverse;
uniform vec3 windSpeed;
uniform float Rotz;

vec2 rotate(vec2 v, float a) {
	float s = sin(a);
	float c = cos(a);
	mat2 m = mat2(c, -s, s, c);
	return m * v;
}

vec2 grassDisplacement(vec4 worldpos, float h)
{
    vec3 FootPos = osg_ViewMatrixInverse[3].xyz;
    vec2 WindVec = max(vec2(windSpeed.z), vec2(windSpeed.z * windSpeed.xy));

    float v = length(WindVec);
    vec2 displace = vec2(2.0 * WindVec + 0.1);
    vec2 harmonics = vec2(0.0);

    harmonics += vec2((1.0 - 0.10*v) * sin(1.0*osg_SimulationTime + /*((h*0.5)*v*WindVec.xy) + */ worldpos.xy / 1100.0));
    harmonics += vec2((1.0 - 0.04*v) * cos(2.0*osg_SimulationTime + /*((h*0.5)*v*WindVec.xy) + */ worldpos.xy / 750.0));
    harmonics += vec2((1.0 + 0.14*v) * sin(3.0*osg_SimulationTime + /*((h*0.5)*v*WindVec.xy) + */ worldpos.xy / 500.0));
    harmonics += vec2((1.0 + 0.28*v) * sin(5.0*osg_SimulationTime + /*((h*0.5)*v*WindVec.xy) + */ worldpos.xy / 200.0));

    float d = length(worldpos.xy - FootPos.xy);
    vec2 stomp = vec2(0.0);
    if(d < 150.0) stomp = (60.0 / d - 0.4) * (worldpos.xy - FootPos.xy);
    return clamp(min(0.004, 1.0/(0.5*(v*v*v))) * h, 0.0, 1.0) * (harmonics * displace + stomp);
}
#endif

void main(void)
{
vec4 viewPos = (gl_ModelViewMatrix * gl_Vertex);

#if @grassAnimation
    vec4 displacedVertex = gl_Vertex;
    vec4 worldPos = osg_ViewMatrixInverse * vec4(viewPos.xyz, 1.0);
    float height = 1.0-(gl_ModelViewMatrix[0].z-gl_Vertex.z);
    vec2 grassVertex = grassDisplacement(worldPos, height);

    if(windSpeed.xy != vec2(0.0)) {
        displacedVertex.z -= length(grassVertex)/3.14;
        grassVertex.xy += height*windSpeed.xy;
    }

    displacedVertex.xy += rotate(grassVertex.xy, -1.0*Rotz);
    gl_Position = (gl_ModelViewProjectionMatrix * displacedVertex);
#else
    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
#endif

    gl_ClipVertex = viewPos;
    euclideanDepth = length(viewPos.xyz);

#if !@radialFog
    linearDepth = gl_Position.z;
#endif

#if @diffuseMap
    diffuseMapUV = (gl_TextureMatrix[@diffuseMapUV] * gl_MultiTexCoord@diffuseMapUV).xy;
#endif

#if !PER_PIXEL_LIGHTING
    vec3 viewNormal = normalize((gl_NormalMatrix * gl_Normal).xyz);
    lighting = doLighting(viewPos.xyz, viewNormal, gl_Color, true);
#else
    passColor = gl_Color;
    passViewPos = viewPos.xyz;
    passNormal = gl_Normal.xyz;
#endif
}
