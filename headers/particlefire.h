/*
* Vulkan Example - CPU based fire particle system 
*
* Copyright (C) 2016 by Sascha Willems - www.saschawillems.de
*
* This code is licensed under the MIT license (MIT) (http://opensource.org/licenses/MIT)
*/

#pragma once

#include <math.h>
#include <vector>

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/matrix_inverse.hpp>

#include <Fade_2D.h>
using namespace GEOM_FADE2D;

#include "vulkanObject.h"

#include "vulkanexamplebase.h"
#include "btBulletDynamicsCommon.h"
#define PARTICLE_COUNT 100
#define PARTICLE_SIZE 10.0f

#define FLAME_RADIUS 0.1f
#define PARTICLE_MASS 0.0000001f

#define PARTICLE_TYPE_FLAME 0
#define PARTICLE_TYPE_SMOKE 1

#define VERTEX_BUFFER_BIND_ID 0

struct Particle {
	glm::vec4 pos;
	glm::vec4 color;
	float alpha;
	float size;
	float rotation;
	uint32_t type;
	// Attributes not used in shader
    glm::vec4 vel;
};

struct BurningPoint{
    glm::vec4 pos;
    uint32_t neighboors[10];
    uint32_t nCount;
    uint32_t state;
};

class VulkanFire : public VulkanObject
{
private:
    VkPipeline propageFire;
    VkPipeline updateParticles;
    VkPipeline clickFire;

    VkQueue computeQueue;
    VkCommandBuffer clickCmd;
    VkPipelineLayout computePipelineLayout;
    VkDescriptorSet computeDescriptorSet;
    VkDescriptorSetLayout computeDescriptorSetLayout;

    VkPipelineLayout pipelineLayout;
    VkDescriptorSetLayout descriptorSetLayout;
    VkDescriptorSet descriptorSet;
    VkPipeline drawParticles;

    struct Attributes{
        VkPipelineVertexInputStateCreateInfo inputState;
        std::vector<VkVertexInputBindingDescription> bindingDescriptions;
        std::vector<VkVertexInputAttributeDescription> attributeDescriptions;
    };

    Attributes attributesParticles;
    Attributes attributesBPoints;

    struct {
        //float deltaT;
        glm::vec4 delta;
        glm::vec4 clickPos;
        int32_t particleCount;
        int32_t bPointsCount;
        int32_t bPointsCount4;
    } computeUbo;

    vkTools::UniformData bPointsStorageBuffer;
    vkTools::UniformData particlesStorageBuffer;
    vkTools::UniformData computeUniformBuffer;

    std::vector<BurningPoint> burningPoints;

    VulkanExampleBase *exampleBase;

    float rnd(float range);
    void initParticle(Particle *particle, int index);
    void reset(Particle *particle);

    void transitionParticle(Particle *particle);
    void prepareParticles(VkQueue);
    // Prepare and initialize uniform buffer containing shader uniforms
    void prepareUniformBuffers();
    void triangulate(std::vector<glm::vec3> data,int i,std::vector<glm::vec3>& allPoints);
    void prepareBurningPoints(VkQueue);
    void prepareComputeLayout(VkDescriptorPool descriptorPool);
    void setupDescriptorSet(VkDescriptorPool pool,VkSampler,vkTools::VulkanTexture fire,vkTools::VulkanTexture smoke);
    void prepareComputePipelines(std::string assetPath);
    void prepareRenderLayout(VkDescriptorPool descriptorPool);
    void prepareRenderPipelines(VkRenderPass,std::string);
    void createClickCommand(VkCommandPool cmdPool);

public:
    void draw(VkCommandBuffer cmdbuffer);
    void compute(VkCommandBuffer cmdbuffer);
    void updateTime(float frameTimer);
    void init(VkQueue queue,VkCommandPool cpool, VkRenderPass renderpass,VkDescriptorPool pool,VkSampler sampler,vkTools::VulkanTexture fire,vkTools::VulkanTexture smoke, std::string path);
    void cliked(VkQueue queue, glm::vec4 pos);
    void addBurningPoints(std::vector<glm::vec3> data);


    VulkanFire(VkDevice device, VulkanExampleBase *example);

    void addToWorld(btDiscreteDynamicsWorld*);

    ~VulkanFire();
};
