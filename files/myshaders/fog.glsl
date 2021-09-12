#ifdef HEIGHT_FOG
varying vec3 fogH;

#if defined(ANIMATED_HEIGHT_FOG) && !defined(PARTICLE)
    const mat2 m = mat2( 1.6,  1.2, -1.2,  1.6 );

vec2 hash( vec2 p ) {
	  p = vec2(dot(p,vec2(127.1,311.7)), dot(p,vec2(269.5,183.3)));
  	return -1.0 + 2.0*fract(sin(p)*43758.5453123);
}

float noise( in vec2 p ) {
    const float K1 = 0.366025404;
    const float K2 = 0.211324865;
	  vec2 i = floor(p + (p.x+p.y)*K1);	
    vec2 a = p - i + (i.x+i.y)*K2;
    vec2 o = (a.x>a.y) ? vec2(1.0,0.0) : vec2(0.0,1.0);
    vec2 b = a - o + K2;
	  vec2 c = a - 1.0 + 2.0*K2;
    vec3 h = max(0.5-vec3(dot(a,a), dot(b,b), dot(c,c) ), 0.0 );
	  vec3 n = h*h*h*h*vec3( dot(a,hash(i+0.0)), dot(b,hash(i+o)), dot(c,hash(i+1.0)));
    return dot(n, vec3(70.0));	
}

float cshadow(in vec2 p, in float oTime) {
	  float time = oTime * 0.02; // speed
	  float f = 0.0;
    vec2 uv = p;
	  uv *= 0.00005; // scale
    float weight = 0.7;
    for (int i=0; i<3; i++){
        f += weight*noise( uv );
        uv = m*uv + time;
		    weight *= 0.6;
    }
	return f;
}
#endif

float CalFade (float fv, float light)
{
#ifdef DYNAMICHFOG
    return clamp(((fv*0.01) * (fv*0.01) * 0.0001)/max(0.1, foghdistance*(light*light*light)), 0.0, 1.0);
#else
	  return clamp(((fv*0.01) * (fv*0.01) * 0.0001)/foghdistance, 0.0, 1.0);
#endif
}

float CalFog (float fv)
{
	  return clamp(exp(-3.3 + 4.0 * fv) -0.2, 0.0, 1.0);
}

float calHFog (float fv)
{
	  return 0.8 * clamp(exp(-7.4 + 10.0 * fv) - 0.05, 0.0, 1.0);
}

float FogMerge(float a, float b)
{
	  return max(a,0.29 * ((a - b) * (a - b) + 2.0) * (a + b));
}
#endif

#if !defined(WATER) && @underwaterFog
float getUnderwaterFogValue(float depth)
{
    float deepValue = abs((osg_ViewMatrixInverse * vec4(passViewPos, 1.0)).z);
    float distFogValue = uwdistfog.z * smoothstep(uwdistfog.x, uwdistfog.y, depth);
    float deepFogValue = uwdeepfog.z * clamp((deepValue - uwdeepfog.x) * (1.0/(uwdeepfog.y-uwdeepfog.x)) , 0.0, 1.0);
        
    return clamp(deepFogValue + distFogValue, 0.0, 1.0);
}
#endif

float getFogValue(float depth)
{
    float fogValue = clamp((depth - gl_Fog.start) * gl_Fog.scale, 0.0, 1.0);

#ifdef HEIGHT_FOG
	      float light = lcalcDiffuse(0).x + lcalcDiffuse(0).y + lcalcDiffuse(0).z;

	      float fogValueH = clamp((depth - 0.0) * 0.001, 0.0, 1.0);
	
	      float ed = CalFog(fogValue);
	      float edfade = CalFade(depth, light);
	      float edhm = calHFog(fogValueH);

        float anim = 1.0;
#if defined(ANIMATED_HEIGHT_FOG) && !defined(PARTICLE)
        if(edfade > 0.2) anim = min(1.0, cshadow((fogH.xy + (1.0 + 100.3 * 1.0/*csh*/)) * 5.0 * vec2(1.0,-1.0), 5.0 * osg_SimulationTime));
        edhm *= smoothstep(0.2, 0.6, edfade);
#endif

//float hcoef = /*1.0 +*/ ((osg_ViewMatrixInverse * vec4(passViewPos, 1.0)).z / 12000.0);


#ifdef DYNAMICHFOG
        float fogheight = fogheight * max(maxfheight,light);
#endif

#ifdef ANIMATED_HEIGHT_FOG
    float ascale = 0.3;
#else
    float ascale = 0.1;
#endif

	      float foghrange = length(fogH.z);
	      foghrange = edhm * clamp(exp(-foghrange * fogheight *  0.0001) ,0.0 , 1.0);
        fogValue = max(fogValue, clamp(mix(1.0 - ascale * anim, 1.0, ed) * edfade * FogMerge(ed, foghrange), 0.0, 1.0));
#endif

    return fogValue;
   // gl_FragData[0].xyz = mix(gl_FragData[0].xyz, gl_Fog.color.xyz, fogValue);
}