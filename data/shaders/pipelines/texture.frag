#version 450

layout (set = 1, binding = 1) uniform sampler2D colorMap;
layout (set = 0,binding = 2) uniform sampler2D burn;
layout (set = 0,binding = 3) uniform samplerCube shadowMaps;
layout (set = 0,binding = 4) buffer Lights 
{
	vec4 lights[];
};

layout (location = 0) in vec2 inUV;
layout (location = 1) in vec3 inNormal;
layout (location = 2) in vec3 worldPos;
layout (location = 3) in float inHeat;

layout (location = 0) out vec4 outFragColor;

layout(push_constant) uniform PushConsts 
{
	uint lIndex;
    float lFactor;
} pushConsts;

#define EPSILON 0.15

void main() 
{
   
    outFragColor = vec4(0.1f,0.1f,0.1f,1.0f);
    
	vec3 N = normalize(inNormal);
    
    vec3 color = texture(colorMap, inUV).rgb;
    vec3 color2 = texture(burn, inUV).rgb;
    color = color*(1.0f-inHeat) + inHeat*color2;
    
    // Shadow
    
    vec3 lightVec = vec3(worldPos - lights[pushConsts.lIndex].xyz);
    vec3 L = normalize(-lightVec);
    float sampledDist = texture(shadowMaps, lightVec).r;
    float dist = length(lightVec);

    // Check if fragment is in shadow
    if(dist <= sampledDist + EPSILON){
        outFragColor.rgb += max(dot(N,L), 0.1) * color;
    }
    outFragColor.rgb*=pushConsts.lFactor;
}
  