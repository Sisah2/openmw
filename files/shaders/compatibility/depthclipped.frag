#version 120

uniform sampler2D diffuseMap;

varying vec2 diffuseMapUV;
varying float alphaPassthrough;
varying vec3 passViewPos;
uniform float near;
uniform float far;
varying float depth;


vec4 pack4 (float depth)
{
    const vec4 bitSh = vec4(256.0 * 256.0 * 256.0, 256.0 * 256.0, 256.0, 1.0);
    const vec4 bitMsk = vec4(0.0, 1.0 / 256.0, 1.0 / 256.0, 1.0 / 256.0);
    vec4 comp = fract(depth * bitSh);
    comp -= comp.xxyz * bitMsk;
    return comp;
}

vec3 pack3 (float depth)
{
    const vec3 bitSh = vec3(256.0 * 256.0, 256.0, 1.0);
    const vec3 bitMsk = vec3(0.0, 1.0 / 256.0, 1.0 / 256.0);
    vec3 comp = fract(depth * bitSh);
    comp -= comp.xxy * bitMsk;
    return comp;
}


void main()
{
    float alpha = texture2D(diffuseMap, diffuseMapUV).a * alphaPassthrough;

    const float alphaRef = 0.499;

    if (alpha < alphaRef)
        discard;

  //  gl_FragData[0] = vec4(gl_FragCoord.z/gl_FragCoord.w/far, 1.0, 0.0, 1.0);



vec3 bitShift3 = vec3(256.0*256.0, 256.0, 1.0);
vec3 bitMask3 = vec3(0.0, 1.0/256.0, 1.0/256.0);
float A = gl_ProjectionMatrix[2].z;
float B = gl_ProjectionMatrix[3].z;
float zNear = - B / (1.0 - A);
float zFar = B / (1.0 + A);
float depthN = (depth - zNear)/(zFar - zNear); // scale to a value in [0, 1]
vec3 depthNPack3 = fract(depthN*bitShift3);
depthNPack3 -= depthNPack3.xxy*bitMask3;
gl_FragData[0]= vec4(depthNPack3, 1.0); // alpha should equal 1.0 if GL_BLEND is enabled

}
