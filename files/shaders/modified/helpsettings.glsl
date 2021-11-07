//#define LINEAR_LIGHTING
//#define ATTEN_FIX

#ifdef LINEAR_LIGHTING
    //dont change this vtastek lighting is made with specific tonemaper in mind!
    #include "glsl-tone-map-master/vtastek.glsl"
#else
    //custom tonemaper can be changed here.
    #include "glsl-tone-map-master/none.glsl"
#endif

// contrast for day and night
// night setting also affects interiors
const float conday = 1.0;
const float connight = 0.75;

//#define HEIGHT_FOG
//#define DYNAMICHFOG
//#define ANIMATED_HEIGHT_FOG
const float fogheight = 5.0;
const float foghdistance = 1.0;
const float maxfheight = 1.0;

// underwater fog setting, need to find better config vec3(start, end, contrib)
const vec3 uwdeepfog = vec3(-2000.0, 1000.0, 1.0); //deeper terrain/objects become more fogged
const vec3 uwdistfog = vec3(-3333.0, 6666.0, 0.15); //distant underwater terrain/objects become more fogged
const vec3 uwfogcolor = vec3(12.0/255.0, 30.0/255.0, 37.0/255.0);
//#define UNDERWATER_DISTORTION

// just fade alpha to make it non-transparent at some distance, allow to hide world edge sometimes
// also darken it a little if using linear lighting; vec2(start, end)
//#define SIMPLE_WATER_TWEAK
vec2 swafader = vec2(-3000.0, 3000.0);

// fade objects normal, specular and env maps at start distance, skip them at end distance
//#define NORMAL_MAP_FADING
const vec2 nmfader = vec2(7455.0, 8196.0);

// some extra grass displacement during storms
#define STORM_MODE
