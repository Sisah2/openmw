OpenMW PBR shaders v05 ALPHA
-linear lighting with ACES and OKLAB tonemappers
-sky lighting prototype
-grass backlighting
-Iterative parallax mapping with soft shadows
-water shaders with waves
-Cook-torrance specular lighting for sun and point lights

Currently, it can only render non metals without ambient reflections.
(metals and reflections after sky and post process shaders.)

_spec is used as the PBR map
this means, delete all your old style spec maps!!!
do not use auto generated maps.

_spec channels
R: metalness 1.0(255) is metal
G: roughness
B: specular, 0.5 for 0.04(conversion is 0.16 * x^2)
A: AO (don't have black alpha)

Conversion from MGE XE param maps:
invert R channel
use G channel as is
turn B channel to 0.5(128/255)
move ao if any to alpha.
if alpha is black, make it white instead.

_normals to _n
invert G channel(openmw is DX style, MGE is OpenGL style. Yes, I know!)

Terrain doesn't have spec,
instead you can put roughness into alpha of _diffusespec
we assume there is no metal ground.

recommendations:
in your openmw.cfg
disable constant and linear lighting instead enable quadratic.
fallback=LightAttenuation_UseQuadratic,1
set sun color to 0,0,0 for overcast weathers
fallback=Weather_Overcast_Sun_Sunrise_Color,001,001,001
fallback=Weather_Overcast_Sun_Day_Color,001,001,001
fallback=Weather_Overcast_Sun_Sunset_Color,001,001,001
fallback=Weather_Overcast_Sun_Night_Color,001,001,001