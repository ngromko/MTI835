#version 450

#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable

layout (location = 0) out float outFragColor;

layout (location = 0) in vec3 inPos;
layout (location = 1) in vec3 inLightPos;

void main() 
{
	// Store distance to light as 32 bit float value
    outFragColor = length(inPos-inLightPos);
}