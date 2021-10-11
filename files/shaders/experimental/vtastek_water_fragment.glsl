#version 120
precision highp float;

#define REFRACTION @refraction_enabled

// Inspired by Blender GLSL Water by martinsh ( https://devlog-martinsh.blogspot.de/2012/07/waterundewater-shader-wip.html )


// tweakables -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --


#define PEAK

const vec3 WATER_COLOR_SHALLOW = 1.0 - vec3(0.1, 0.75, 0.7);
const vec3 WATER_COLOR_DEEP = vec3(0.05,0.3,0.5);

const float dmeter = 200.0;

const float VISIBILITY = 0.000;

const float BIG_WAVES_X = 0.2; // strength of big waves
const float BIG_WAVES_Y = 0.3;

const float MID_WAVES_X = 0.9; // strength of middle sized waves
const float MID_WAVES_Y = 0.3;
const float MID_WAVES_RAIN_X = 0.95;
const float MID_WAVES_RAIN_Y = 0.99;

const float SMALL_WAVES_X = 0.1; // strength of small waves
const float SMALL_WAVES_Y = 0.1;
const float SMALL_WAVES_RAIN_X = 0.3;
const float SMALL_WAVES_RAIN_Y = 0.3;

const float WAVE_CHOPPYNESS = 0.2;                // wave choppyness
const float WAVE_SCALE = 175.0;                     // overall wave scale

const float BUMP = 0.35;                            // overall water surface bumpiness
const float BUMP_RAIN = 0.28;
const float REFL_BUMP = 0.03;                      // reflection distortion amount
const float REFR_BUMP = 0.03;                      // refraction distortion amount

const float SCATTER_AMOUNT = 18.0;                  // amount of sunlight scattering
const vec3 SCATTER_COLOUR = vec3(0.0,1.0,0.95);    // colour of sunlight scattering

const vec3 SUN_EXT = vec3(0.45, 0.55, 0.68);       //sunlight extinction

const float SPEC_HARDNESS = 255.0;                 // specular highlights hardness

const float BUMP_SUPPRESS_DEPTH = 300.0;           // at what water depth bumpmap will be suppressed for reflections and refractions (prevents artifacts at shores)

const vec2 WIND_DIR = vec2(0.5f, -0.8f);
const float WIND_SPEED = 0.2f;

const vec3 WATER_COLOR = vec3(0.090195, 0.115685, 0.12745);

// ---------------- rain ripples related stuff ---------------------

const float RAIN_RIPPLE_GAPS = 16.0;
const float RAIN_RIPPLE_RADIUS = 0.3;

vec2 randOffset(vec2 c)
{
  return fract(vec2(
          c.x * c.y / 8.0 + c.y * 0.3 + c.x * 0.2,
          c.x * c.y / 14.0 + c.y * 0.5 + c.x * 0.7));
}

float randPhase(vec2 c)
{
  return fract((c.x * c.y) /  (c.x + c.y + 0.1));
}

vec4 circle(vec2 coords, vec2 i_part, float phase, float randomp)
{
  vec2 center = vec2(0.5,0.5) + (0.5 - RAIN_RIPPLE_RADIUS) * (2.0 * randOffset(i_part) - 1.0);
  vec2 toCenter = coords - center;
  float d = length(toCenter);

  float r = RAIN_RIPPLE_RADIUS * phase;

  if (d > r)
    return vec4(0.0,0.0,1.0,0.0);

  float sinValue = (sin(d / r * 1.0) + 1.2) / 2.0;

  float height = (1.0 - abs(phase)) * pow(sinValue,66.0);
  float height2 = (1.0 - abs(phase*0.01)) * pow(sinValue, 2.0);
  height2 = sin(height2 * 3.14 * 36.0 / (sin(phase) + 3.0));
  
  height2 = mix(height2, 0.0, sin(phase) + 0.2);

  vec3 normal = normalize(mix(vec3(0.0,0.0,1.0),vec3(normalize(toCenter),0.0),height2));

  return vec4(normal,height);
}

vec4 rain(vec2 uv, float time)
{
  vec2 i_part = floor(uv * RAIN_RIPPLE_GAPS);
  vec2 f_part = fract(uv * RAIN_RIPPLE_GAPS);
 //return vec4( 3.0 * fract(time * 0.17 + randPhase(i_part)) - 2.0);
  return circle(f_part,i_part, 5.0 * fract(time * 0.07 + randPhase(i_part)) - 4.0, randPhase(i_part));
}

vec4 rainCombined(vec2 uv, float time)     // returns ripple normal in xyz and ripple height in w
{
	float rtime = time * 0.001;
  return
    //rain(uv,time) ;
    rain(uv + fract(mod(rtime,11.0)) + vec2(10.5,5.7),time + 197.0) + 1.0 *
	rain(uv + fract(mod(rtime,31.0)) + vec2(40.6,35.7),time + 297.0) +
	rain(uv + fract(mod(rtime,11.0)) + vec2(30.5,25.7),time + 397.0) +
	rain(uv + fract(mod(rtime,41.0)) + vec2(14.69,9.3),time + 97.0) +
    rain(uv + fract(mod(rtime,2.0)) + vec2(31.9,4.8),time + 727.0)+
    rain(uv + fract(mod(rtime,23.0)) + vec2(13.2,7.7),time + 691.0)+
    rain(uv + fract(mod(rtime,3.0)) + vec2(20.3,3.6),time + 701.0) +
    rain(uv * 0.75 + fract(mod(rtime,29.0)) + vec2(3.7,18.9),time + 281.0) +
    rain(uv * 0.9 +  fract(mod(rtime,7.0)) + vec2(5.7,30.1),time + 863.0) +
    rain(uv * 0.8 + fract(mod(rtime,31.0)) + vec2(1.2,3.0),time + 317.0);
}

// -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -

float fresnel_dielectric(vec3 Incoming, vec3 Normal, float eta)
  {
    float c = abs(dot(Incoming, Normal));
    float g = eta * eta - 1.0 + c * c;
    float result;

    if(g > 0.0) {
        g = sqrt(g);
        float A =(g - c)/(g + c);
        float B =(c *(g + c)- 1.0)/(c *(g - c)+ 1.0);
        result = 0.5 * A * A *(1.0 + B * B);
    }
    else
        result = 1.0;  /* TIR (no refracted component) */

    return result;
  }

vec2 normalCoords(vec2 uv, float scale, float speed, float time, float timer1, float timer2, vec3 previousNormal, vec2 dir)
  {
    return uv * (WAVE_SCALE * scale) + dir * time * (WIND_SPEED * speed) -(previousNormal.xy/previousNormal.zz) * WAVE_CHOPPYNESS + vec2(time * timer1,time * timer2);
  }
  
vec3 GetWaterColor(float accumulatedWater, float depth, vec3 refractionValue, vec3 incidentLight)
{
   
   vec3 refractionAmountAtSurface = refractionValue * exp(-WATER_COLOR_SHALLOW * (depth + accumulatedWater));
   
																				 
   
   float inverseScatterAmount = exp(-0.09 * accumulatedWater);

   //return vec3(inverseScatterAmount);
   return mix(incidentLight * WATER_COLOR_DEEP * 0.25, refractionAmountAtSurface * incidentLight, clamp(inverseScatterAmount, 0.0, 1.0)) * incidentLight * 0.75;
}


varying vec3 screenCoordsPassthrough;
varying vec4 position;
varying float linearDepth;

uniform sampler2D normalMap;

uniform sampler2D reflectionMap;
#if REFRACTION
uniform sampler2D refractionMap;
uniform highp sampler2D refractionDepthMap;
#endif

uniform float osg_SimulationTime;

uniform float near;
uniform float far;
uniform vec3 nodePosition;

uniform float rainIntensity;

#include "shadows_fragment.glsl"

float frustumDepth;

float linearizeDepth(float depth)
  {
    float z_n = 2.0 * depth - 1.0;
    depth = 2.0 * near * far / (far + near - z_n * frustumDepth);
    return depth;
  }

							  
 
												 
 uniform bool radialFog;
 uniform float gamma;

void main(void)
{
    frustumDepth = abs(far - near);
    vec3 worldPos = position.xyz + nodePosition.xyz;
    vec2 UV = worldPos.xy / (8192.0 * 5.0) * 3.0;
    UV.y *= -1.0;
	
	vec2 dir = vec2(1.0,-1.0) * normalize(worldPos.xy + vec2(-20000.0,-69000.0));																						 

	vec3 cameraPos = (gl_ModelViewMatrixInverse * vec4(0.0,0.0,0.0,1.0)).xyz;
    
	float shadow = unshadowedLightRatio(linearDepth);

    vec2 screenCoords = screenCoordsPassthrough.xy / screenCoordsPassthrough.z;
	
    vec2 screenCoordsref = screenCoordsPassthrough.xy / screenCoordsPassthrough.z;
	
    screenCoords.y = (1.0 - screenCoords.y);
    screenCoordsref.y = (1.0-screenCoordsref.y);
	
    #define waterTimer osg_SimulationTime * 3.14

	vec2 windir = WIND_DIR;
    vec3 normal0 = 2.0 * texture2D(normalMap,normalCoords(UV, 0.05, 0.04, waterTimer, -0.015, -0.005, vec3(0.0,0.0,0.0), windir)).rgb - 1.0;
    vec3 normal1a = 2.0 * texture2D(normalMap,normalCoords(UV, 0.02,  0.05, 0.1 * waterTimer,  0.0,   0.0, normal0, windir * 1.0)).rgb - 1.0;
    vec3 normal1 = 2.0 * texture2D(normalMap,normalCoords(UV, 0.001,  0.01, 0.0001 * waterTimer,  0.0,   0.0, normal1a * 1.0, 0.1 * dir)).rgb - 1.0;
    vec3 normal2 = 2.0 * texture2D(normalMap,normalCoords(UV, 0.25, 0.01, waterTimer, -0.04,  -0.03,  normal1,windir)).rgb - 1.0;
    vec3 normal3 = 2.0 * texture2D(normalMap,normalCoords(UV, 0.5,  0.09, waterTimer,  0.03,   0.04,  normal2,windir)).rgb - 1.0;
    /*vec3 normal4 = 2.0 * texture2D(normalMap,normalCoords(UV, 1.0,  0.4,  waterTimer, -0.02,   0.1,   normal3,windir)).rgb - 1.0;
    vec3 normal5 = 2.0 * texture2D(normalMap,normalCoords(UV, 2.0,  0.7,  waterTimer,  0.02, -0.04,  normal4,windir)).rgb - 1.0;
    vec3 normal6 = 2.0 * texture2D(normalMap,normalCoords(UV, 2.0,  0.7,  waterTimer,  -0.02, 0.2,  normal5,windir)).rgb - 1.0;
	*/
    vec4 rainRipple;

    if (rainIntensity > 0.01)
      rainRipple = rainCombined(position.xy / 1000.0,waterTimer) * clamp(rainIntensity,0.0,1.0);
    else
      rainRipple = vec4(0.0);
	
	vec4 rainRipplet = rainRipple; 
	
    vec3 rippleAdd = normalize(rainRipple.xyz * rainRipple.w * 10.0 * vec3(1.0,1.0,0.03));
	
	rippleAdd = clamp(rippleAdd, 0.0, 1.0);
	
	float rainnorm = sin(rainRipple.w);
	
	//rippleAdd = normalize(vec3(-vec2(dFdx(rainnorm), dFdy(rainnorm)),0.0)); 

    vec2 bigWaves = vec2(BIG_WAVES_X,BIG_WAVES_Y);
    vec2 midWaves = mix(vec2(MID_WAVES_X,MID_WAVES_Y),vec2(MID_WAVES_RAIN_X,MID_WAVES_RAIN_Y),rainIntensity);
    vec2 smallWaves = mix(vec2(SMALL_WAVES_X,SMALL_WAVES_Y),vec2(SMALL_WAVES_RAIN_X,SMALL_WAVES_RAIN_Y),rainIntensity);
    float bump = mix(BUMP,BUMP_RAIN,rainIntensity);
	float dist = length(position.xy - cameraPos.xy);
	float distz = length(position.xyz - cameraPos.xyz);
	bump = mix(bump, 1.0, clamp(dist/2000.0, 0.0, 1.0));
    //vec3 normal = normal1 * bigWaves.y + normal2 * midWaves.x + normal3 * midWaves.y;
    vec3 normal = normal1 * bigWaves.y + normal2 * midWaves.x  +  normal3 * midWaves.y + rippleAdd.xyz;
    //vec3 normal = normal1;
	
	

//	+ normal2 * midWaves.x + normal3 * midWaves.y + normal4 * smallWaves.x  + normal5 * smallWaves.y + 
	//normal6 * smallWaves.y + 
	//rippleAdd.xyz;

/*	(normal0 * bigWaves.x + normal1 * bigWaves.y + normal2 * midWaves.x +
                   normal3 * midWaves.y + normal4 * smallWaves.x + normal5 * smallWaves.y + rippleAdd);
				   */
    normal = normalize(vec3(-normal.x, -normal.y, normal.z * bump));
	
    vec3 lVec = normalize((gl_ModelViewMatrixInverse * vec4(gl_LightSource[0].position.xyz, 0.0)).xyz);

    
    vec3 vVec = normalize(position.xyz - cameraPos.xyz);
	
	float waterdif = dot(normal, lVec);

    float sunFade = length(gl_LightSource[0].diffuse.xyz + 0.33 * gl_LightModel.ambient.xyz);

    // fresnel
    float ior = (cameraPos.z>0.0)?(1.333/1.0):(1.0/1.333); // air to water; water to air
	float fresnelbias = (cameraPos.z>0.0)?(1.0):(0.0);
    float fresnel = clamp(fresnel_dielectric(vVec, normal, ior), 0.0, 1.0) + fresnelbias * 0.2 ;

    float radialise = 1.0;

float radialDepth;
if(radialFog)
    radialDepth = distance(position.xyz, cameraPos);

    vec2 screenCoordsOffset = normal.xy * REFL_BUMP;
	
	vec2 screenCoordsOffsetRef = normal.xy * REFR_BUMP;
#if REFRACTION

	
    float depthSample = linearizeDepth(texture2D(refractionDepthMap,screenCoords - screenCoordsOffsetRef).x) * radialise;
	float surfaceDepth = linearizeDepth(gl_FragCoord.z) * radialise;
    float realWaterDepth = depthSample - surfaceDepth;  // undistorted water depth in view direction, independent of frustum
	
	
	float suppressfix = clamp(min(smoothstep(0.0,0.8,(realWaterDepth)/100.0) , 1.0 - pow(distz/2000.0, 10.0)), 0.0, 1.0);
	
	//screenCoordsOffsetRef *= suppressfix;
	
	
	//float ndepth = linearizeDepth(texture2D(refractionDepthMap,screenCoords).x);
	//ndepth *= 0.0005;
    float depthSampleDistorted = linearizeDepth(texture2D(refractionDepthMap,screenCoords-screenCoordsOffsetRef * suppressfix).x) * radialise;
    
    //screenCoordsOffset *= clamp(realWaterDepth / BUMP_SUPPRESS_DEPTH,0.0,1.0);
#endif
    // reflection
    vec3 reflection = texture2D(reflectionMap, screenCoords + screenCoordsOffset).rgb;
	
	vec3 realreflection = reflection;

    // specular
	float specular = 10.0 * pow(max(dot(reflect(vVec, normal), lVec), 0.0),SPEC_HARDNESS) * 1.0;

    vec3 waterColor = WATER_COLOR * sunFade;
	
	

#if REFRACTION
    // refraction
	vec3 flatgeo = normalize(normal * vec3(1.0, 1.0, 70000.0));
	//float depthreal = realWaterDepth * (dot(vVec, - flatgeo));
	//float suppress = smoothstep(0.0,0.05,(realWaterDepth)/500.0);
	
	
  	
	float depthSampleDistortedfix = linearizeDepth(texture2D(refractionDepthMap, screenCoords - screenCoordsOffsetRef * suppressfix).x) * radialise;
	 
	 float realWaterDepthfix = clamp(depthSampleDistortedfix - surfaceDepth, 0.0, 5000.0);
	 
	 
	float realsuppressfix = clamp(min(smoothstep(0.0, 0.8, (realWaterDepthfix)/100.0), 1.0 - pow(distz/2000.0, 10.0)), 0.0, 1.0);
	
	screenCoordsOffsetRef *= realsuppressfix;
	 
	 //float suppressfix = smoothstep(0.0,0.05,(realWaterDepthfix)/500.0);
	 
	 float depthrealfix = realWaterDepthfix * (dot(vVec, - flatgeo));
	 
	 float smoothshores = smoothstep(0.0,0.05 + length(normal.xy) * 20.0,depthrealfix/1.0);
	
	float smoothshoresfix = pow((cameraPos.z + dist)/8000.0, 5.0);
	smoothshoresfix = max(smoothshoresfix, smoothshores);
	 
	vec3 refraction = texture2D(refractionMap, screenCoords - screenCoordsOffsetRef).rgb;
	vec3 realrefraction = refraction;
	 
	waterColor = GetWaterColor( realWaterDepthfix/dmeter, depthrealfix/dmeter, refraction, gl_LightSource[0].diffuse.xyz + gl_LightModel.ambient.xyz);
	
	//waterColor *= clamp(waterdif + 0.6, 0.0, 1.0);
    // brighten up the refraction underwater
    if (cameraPos.z < 0.0)
        refraction = clamp(refraction * 1.3, 0.0, 1.0);
    else
        refraction = waterColor; //mix(waterColor, refraction, clamp(exp(-depthSampleDistorted * VISIBILITY), 0.0, 1.0));

    // sunlight scattering
    // normal for sunlight scattering
    /*vec3 lNormal = normal0 * bigWaves.x + normal1 * bigWaves.y + normal2 * midWaves.x + normal3 * midWaves.y + normal4 * smallWaves.x  + normal5 * smallWaves.y + 
	normal6 * smallWaves.y + 
	rippleAdd;
    lNormal = normalize(vec3(-lNormal.x, -lNormal.y, lNormal.z * bump));
	*/
    float sunHeight = lVec.z;
    vec3 scatterColour = mix(SCATTER_COLOUR*vec3(1.0,0.4,0.0), SCATTER_COLOUR, clamp(1.0 - exp(-sunHeight*SUN_EXT), 0.0, 1.0));
	scatterColour *= 1.0 * gl_LightSource[0].diffuse.xyz;
    vec3 lR = reflect(lVec, flatgeo);
    float lightScatter =  1.0 - clamp(dot(lVec,normal),0.0, 1.0);
	lightScatter *= clamp(dot(lR, vVec) * 2.0 - 1.5, 0.0, 1.0) * sunFade * clamp(1.0-exp(-sunHeight), 0.0, 1.0);
	
	fresnel = clamp(fresnel, 0.0, 1.0);
	
	//lightScatter = pow((1.0 - dot(vVec, -flatgeo)), 2.0) *  pow(lightScatter, 1.0);
	
	
	lightScatter = 1.0 - clamp(dot(lVec,normal),0.0, 1.0);;
	lightScatter *= clamp(shadow + 0.25, 0.0, 1.0) * sunFade * max(0.0, dot(lR, vVec)); 
	lightScatter *= clamp(exp(-sunHeight), 0.0, 1.0); 
	
	//clamp(dot(lVec,normal)*0.7 + 0.3, 0.0, 1.0);
	
	//* clamp(dot(lR, vVec)*2.0-1.2, 0.0, 1.0) * SCATTER_AMOUNT * sunFade * clamp(1.0-exp(-sunHeight), 0.0, 1.0);
   //float lightScatter = clamp(1.0 - dot(lVec,normal)*0.7 + 0.3, 0.0, 1.0) * clamp(dot(lR, vVec)*2.0-1.2, 0.0, 1.0) * SCATTER_AMOUNT * sunFade * clamp(1.0-exp(-sunHeight), 0.0, 1.0);
   
   vec3 reflectiontog = mix(vec3(1.0),reflection, fresnelbias); 
   
   
   
   vec3 a = mix(refraction * clamp(waterdif + 0.3, 0.0, 1.0), refraction * scatterColour * SCATTER_AMOUNT, lightScatter);
   //if(cameraPos.z < 0.0)
	//reflection =  reflection * clamp(0.0, 1.0, waterdif + 0.3) + scatterColour * SCATTER_AMOUNT * lightScatter * 1000.0;
   
   vec3 b = mix(refraction * fresnel, reflection, clamp(reflectiontog * 1.66, 0.0, 1.0));
   
   
   gl_FragData[0].xyz = mix( a, b ,  fresnel) + shadow * specular * gl_LightSource[0].diffuse.xyz * gl_LightSource[0].diffuse.xyz + clamp(dist/6000.0,0.0,0.2) * clamp(vec3(rainRipple.w * (gl_LightSource[0].diffuse.xyz + gl_LightModel.ambient.xyz)), 0.0, 1.0);
   
   
   
   
   
    gl_FragData[0].w = 1.0;
#else

	vec3 waterColorOff = GetWaterColor( 1.0, 1.0, vec3(1.0), gl_LightSource[0].diffuse.xyz + gl_LightModel.ambient.xyz);


    gl_FragData[0].xyz = mix(reflection,  waterColor,  (1.0-fresnel)*0.5) + specular * gl_LightSource[0].specular.xyz;
    gl_FragData[0].w = clamp(fresnel + specular * gl_LightSource[0].specular.w, 0.0, 1.0);     //clamp(fresnel*2.0 + specular * gl_LightSource[0].specular.w, 0.0, 1.0);
#endif
	gl_FragData[0].xyz = gl_FragData[0].xyz / (gl_FragData[0].xyz + 1.0);
	
	vec2 underwaterfogfix = mix(vec2(-10000.0, 3.0), vec2(0.0, 1.0), fresnelbias);
	
	
    // fog
float fogValue;
if(radialFog)
    fogValue = clamp((radialDepth - gl_Fog.start + underwaterfogfix.x) * gl_Fog.scale * underwaterfogfix.y, 0.0, 1.0);
else
    fogValue = clamp((linearDepth - gl_Fog.start) * gl_Fog.scale, 0.0, 1.0);

	#if REFRACTION
    //gl_FragData[0].xyz = mix(realrefraction * 1.0, gl_FragData[0].xyz, max(pow(clamp(dist/3000.0, 0.0, 1.0), 2.0), smoothshores));
	
	smoothshores = mix(1.0, smoothshores, fresnelbias);
    gl_FragData[0].xyz = mix(clamp((1.0 + 0.3 + gl_LightModel.ambient.xyz * 0.33) ,0.0, 1.0) * realrefraction * 1.0, gl_FragData[0].xyz, smoothshores);


	#ifdef PEAK
	
	float distline = dist + length(normal.xy) * 17.0;
		
	if (cameraPos.z < 5.0 && cameraPos.z > 0.0 && distline < 40.0) {
	gl_FragData[0].xyz = refraction + 0.1 * a;
	float line = clamp(0.5 + 2.0 * abs(smoothstep(36.0, 43.0, distline) - 0.5), 0.0, 1.0);
	gl_FragData[0].xyz *= vec3(line);
	}
	
	#endif
	
	#else
	vec3 wcoloff = mix(WATER_COLOR_DEEP, 1.0 - WATER_COLOR_SHALLOW, fresnel);
	gl_FragData[0].xyz = vec3(wcoloff * 0.3);

	gl_FragData[0].xyz += clamp(mix(vec3(0.0), 1.0 - WATER_COLOR_SHALLOW, length(wcoloff) * 0.0 + 1.0 - 5.0 * fresnel), 0.0, 1.0);
	//gl_FragData[0].xyz = vec3(1.0 - length(wcoloff) * 0.1 + fresnel);
	
	vec3 reflectiontog = mix(vec3(1.0),reflection, fresnelbias); 
	vec3 b = mix(vec3(0.5 * (gl_LightSource[0].diffuse.xyz + gl_LightModel.ambient.xyz)* gl_FragData[0].xyz), reflection, reflectiontog);
	
	reflection = mix(reflection, b, fresnel * 0.5);
	
	gl_FragData[0].xyz = 0.5 * (gl_LightSource[0].diffuse.xyz + gl_LightModel.ambient.xyz) * mix(gl_FragData[0].xyz, reflection, max(0.0, fresnel - 0.15)) + specular * gl_LightSource[0].specular.xyz;
	gl_FragData[0].w = dist * 0.00005 + length(1.0 - gl_FragData[0].xyz) * 0.4 + pow(fresnel,1.0/1.0) + specular * gl_LightSource[0].specular.w;
	
	#endif
	
	gl_FragData[0].xyz = mix(gl_FragData[0].xyz,  gl_Fog.color.xyz, fogValue);
   
	//debugs
	#if REFRACTION
	//gl_FragData[0].xyz = vec3(max(pow(clamp(dist/3000.0, 0.0, 1.0), 2.0), smoothshores));
	//"gl_FragData[0].xyz = vec3(a);
	//gl_FragData[0].xyz = vec3(realreflection);
	//gl_FragData[0].xyz = vec3(waterColor);
	//gl_FragData[0].xyz = vec3(lightScatter);
	//gl_FragData[0].xyz = vec3(smoothshores);
	//gl_FragData[0].xyz = vec3(fresnel);
	//gl_FragData[0].xyz = vec3(realWaterDepth/5000.0);
	//gl_FragData[0].xyz = vec3(ndepth);
	//gl_FragData[0].xyz = vec3(rippleAdd.xyz);
	#endif

    applyShadowDebugOverlay();

gl_FragData[0].xyz = pow(gl_FragData[0].xyz, vec3(1.0/gamma));

}
