#pragma once

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/matrix_inverse.hpp>

#include "vulkan/vulkan.h"
#include "vulkantools.h"
#include "vulkanexamplebase.h"

#include "btBulletDynamicsCommon.h"

class VulkanObject
{
protected:
    VkDevice device;
    VulkanExampleBase *exampleBase;
    uint8_t *pBurn;
    uint32_t offset;

    vkTools::UniformData uniformData;

    VkDescriptorSet descriptorSet;

    struct{
        glm::mat4 projection;
        glm::mat4 model;
        glm::mat4 view;
    } ubo;
    btRigidBody* rbody;

    vkTools::VulkanTexture* texture;

    void prepareUniformBuffer(glm::mat4 model)
    {
        ubo.model= model;

        exampleBase->createBuffer(
            VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
            sizeof(ubo),
            &ubo,
            &uniformData.buffer,
            &uniformData.memory,
            &uniformData.descriptor);
    }

public:
    virtual void updateModel(glm::mat4 model){
        ubo.model = model;
        updateUniformBuffer();
        //std::cout<<"model " << offset << std::endl;
        memcpy(pBurn+offset, &model, sizeof(model));
    }

    void updateProjView(glm::mat4 projection, glm::mat4 view){
        ubo.projection = projection;
        ubo.view = view;
        updateUniformBuffer();
    }
    VulkanObject(VkDevice mdevice, VulkanExampleBase *mexample, vkTools::VulkanTexture *eTexture) : device(mdevice), exampleBase(mexample), texture(eTexture){

    }

    ~VulkanObject(){
        vkDestroyBuffer(device, uniformData.buffer, nullptr);
        vkFreeMemory(device, uniformData.memory, nullptr);
    }

    btRigidBody* getRigidBody(){
        return rbody;
    }

    void setupDescriptorSet(VkDescriptorPool pool, VkDescriptorSetLayout descriptorSetLayout){
        VkDescriptorSetAllocateInfo allocInfo =
            vkTools::initializers::descriptorSetAllocateInfo(
                pool,
                &descriptorSetLayout,
                1);

        VkResult vkRes = vkAllocateDescriptorSets(device, &allocInfo, &descriptorSet);
        assert(!vkRes);

        // Color map image descriptor
        VkDescriptorImageInfo texDescriptorColorMap =
            vkTools::initializers::descriptorImageInfo(
                texture->sampler,
                texture->view,
                VK_IMAGE_LAYOUT_GENERAL);

        std::vector<VkWriteDescriptorSet> writeDescriptorSets =
        {
            // Binding 0 : Vertex shader uniform buffer
            vkTools::initializers::writeDescriptorSet(
            descriptorSet,
                VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
                0,
                &uniformData.descriptor),
            // Binding 1 : Fragment shader image sampler
            vkTools::initializers::writeDescriptorSet(
                descriptorSet,
                VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                1,
                &texDescriptorColorMap)
        };

        vkUpdateDescriptorSets(device, writeDescriptorSets.size(), writeDescriptorSets.data(), 0, NULL);
    }

    void setupDescriptorSet(VkDescriptorPool pool, VkDescriptorSetLayout descriptorSetLayout, uint32_t offSet, uint8_t* pdata)
    {
        offset = offSet;
        this->pBurn = pdata;
        setupDescriptorSet(pool,descriptorSetLayout);
        updateModel(ubo.model);
    }

    virtual void draw(VkCommandBuffer cmdbuffer, VkPipelineLayout pipelineLayout)=0;

private:
    void updateUniformBuffer(){
        uint8_t *pData;
        VkResult err = vkMapMemory(device, uniformData.memory, 0, sizeof(ubo), 0, (void **)&pData);
        assert(!err);
        memcpy(pData, &ubo, sizeof(ubo));
        vkUnmapMemory(device, uniformData.memory);
    }
};
