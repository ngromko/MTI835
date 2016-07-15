#version 450

layout (binding = 1) uniform sampler2D colorMap;
layout (binding = 2) uniform sampler2D burn;

layout (location = 0) in vec2 inUV;
layout (location = 1) in vec3 inNormal;
layout (location = 2) in float inHeat;

layout (location = 0) out vec4 outFragColor;

void main() 
{
	
	vec3 N = normalize(inNormal);
	vec3 L = normalize(vec3(2.0, 2.0, 2.0));
	
	vec3 color = texture(colorMap, inUV).rgb;
    vec3 color2 = texture(burn, inUV).rgb;
    color = color*(1.0f-inHeat) + inHeat*color2;
	outFragColor.rgb = vec3(clamp(max(dot(N,L), 0.0), 0.15, 1.0)) * color; 	
}
  