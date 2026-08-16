#version 120

#include "lib/core/vertex.h.glsl"

void main(void)
{
    gl_Position = modelToClip(gl_Vertex);
    applyClipPlanes(modelToView(gl_Vertex));
}