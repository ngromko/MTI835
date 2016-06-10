/*
* Vulkan Example - Using different pipelines in one single renderpass
*
* Copyright (C) 2016 by Sascha Willems - www.saschawillems.de
*
* This code is licensed under the MIT license (MIT) (http://opensource.org/licenses/MIT)
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <iostream>
#include <vector>

#include <Fade_2D.h>

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <gli/gli.hpp>

#include <vulkan/vulkan.h>

#include "vulkanexamplebase.h"

#include "btBulletDynamicsCommon.h"
#include "vulkancube.h"
#include "particlefire.h"

#define VERTEX_BUFFER_BIND_ID 0
#define ENABLE_VALIDATION false

using namespace GEOM_FADE2D;
class VulkanExample: public VulkanExampleBase 
{
private:
	vkTools::VulkanTexture textureColorMap;
    std::chrono::time_point<std::chrono::high_resolution_clock> tStart;
    btRigidBody* selectBody;
    float time = 0;

public:
	struct {
		VkPipelineVertexInputStateCreateInfo inputState;
		std::vector<VkVertexInputBindingDescription> bindingDescriptions;
		std::vector<VkVertexInputAttributeDescription> attributeDescriptions;
	} vertices;
    struct {
        VkPipelineVertexInputStateCreateInfo inputState;
        std::vector<VkVertexInputBindingDescription> bindingDescriptions;
        std::vector<VkVertexInputAttributeDescription> attributeDescriptions;
        struct {
            vkTools::VulkanTexture smoke;
            vkTools::VulkanTexture fire;
            // We use a custom sampler to change some sampler
            // attributes required for rotation the uv coordinates
            // inside the shader for alpha blended textures
            VkSampler sampler;
        } textures;
    } particles;

    btDiscreteDynamicsWorld* dynamicsWorld;

    std::vector<VulkanCube*> cubes;
    VulkanFire* fire;

	VkPipelineLayout pipelineLayout;
	VkDescriptorSet descriptorSet;
	VkDescriptorSetLayout descriptorSetLayout;

	struct {
		VkPipeline solidColor;
		VkPipeline wireFrame;
		VkPipeline texture;
        VkPipeline fire;
	} pipelines;

	VulkanExample() : VulkanExampleBase(ENABLE_VALIDATION)
	{
		zoom = -5.0f;
        rotation = glm::vec3(0.0f, 0.0f, 0.0f);
		title = "Vulkan Example - Using pipelines";
	}

	~VulkanExample()
	{
		// Clean up used Vulkan resources 
		// Note : Inherited destructor cleans up resources stored in base class
		vkDestroyPipeline(device, pipelines.solidColor, nullptr);
        //vkDestroyPipeline(device, pipelines.wireFrame, nullptr);
        //vkDestroyPipeline(device, pipelines.texture, nullptr);


        vkDestroyPipeline(device, pipelines.fire, nullptr);

        vkDestroySampler(device, particles.textures.sampler, nullptr);
		
		vkDestroyPipelineLayout(device, pipelineLayout, nullptr);
        vkDestroyDescriptorSetLayout(device, descriptorSetLayout, nullptr);

		textureLoader->destroyTexture(textureColorMap);
        textureLoader->destroyTexture(particles.textures.smoke);
        textureLoader->destroyTexture(particles.textures.fire);
	}

	void loadTextures()
	{
        textureLoader->loadTexture(
			getAssetPath() + "textures/crate_bc3.ktx", 
			VK_FORMAT_BC3_UNORM_BLOCK, 
			&textureColorMap);

        // Particles
        textureLoader->loadTexture(
            getAssetPath() + "textures/particle_smoke.ktx",
            VK_FORMAT_BC3_UNORM_BLOCK,
            &particles.textures.smoke);
        textureLoader->loadTexture(
            getAssetPath() + "textures/particle_fire.ktx",
            VK_FORMAT_BC3_UNORM_BLOCK,
            &particles.textures.fire);

        // Create a custom sampler to be used with the particle textures
        // Create sampler
        VkSamplerCreateInfo samplerCreateInfo = vkTools::initializers::samplerCreateInfo();
        samplerCreateInfo.magFilter = VK_FILTER_LINEAR;
        samplerCreateInfo.minFilter = VK_FILTER_LINEAR;
        samplerCreateInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
        // Different address mode
        samplerCreateInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
        samplerCreateInfo.addressModeV = samplerCreateInfo.addressModeU;
        samplerCreateInfo.addressModeW = samplerCreateInfo.addressModeU;
        samplerCreateInfo.mipLodBias = 0.0f;
        samplerCreateInfo.compareOp = VK_COMPARE_OP_NEVER;
        samplerCreateInfo.minLod = 0.0f;
        // Both particle textures have the same number of mip maps
        samplerCreateInfo.maxLod = particles.textures.fire.mipLevels;
        // Enable anisotropic filtering
        samplerCreateInfo.maxAnisotropy = 8;
        samplerCreateInfo.anisotropyEnable = VK_TRUE;
        // Use a different border color (than the normal texture loader) for additive blending
        samplerCreateInfo.borderColor = VK_BORDER_COLOR_FLOAT_TRANSPARENT_BLACK;
        VkResult err = vkCreateSampler(device, &samplerCreateInfo, nullptr, &particles.textures.sampler);
        assert(!err);
	}

    void buildCommandBuffers()
    {
        VkCommandBufferBeginInfo cmdBufInfo = vkTools::initializers::commandBufferBeginInfo();

        VkClearValue clearValues[2];
        clearValues[0].color = defaultClearColor;
        clearValues[1].depthStencil = { 1.0f, 0 };

        VkRenderPassBeginInfo renderPassBeginInfo = vkTools::initializers::renderPassBeginInfo();
        renderPassBeginInfo.renderPass = renderPass;
        renderPassBeginInfo.renderArea.offset.x = 0;
        renderPassBeginInfo.renderArea.offset.y = 0;
        renderPassBeginInfo.renderArea.extent.width = width;
        renderPassBeginInfo.renderArea.extent.height = height;
        renderPassBeginInfo.clearValueCount = 2;
        renderPassBeginInfo.pClearValues = clearValues;

        VkResult err;

        for (int32_t i = 0; i < drawCmdBuffers.size(); ++i)
        {
            // Set target frame buffer
            renderPassBeginInfo.framebuffer = frameBuffers[i];

            err = vkBeginCommandBuffer(drawCmdBuffers[i], &cmdBufInfo);
            assert(!err);

            vkCmdBeginRenderPass(drawCmdBuffers[i], &renderPassBeginInfo, VK_SUBPASS_CONTENTS_INLINE);

            VkViewport viewport = vkTools::initializers::viewport(
                (float)width,
                (float)height,
                0.0f,
                1.0f);
            vkCmdSetViewport(drawCmdBuffers[i], 0, 1, &viewport);

            VkRect2D scissor = vkTools::initializers::rect2D(
                width,
                height,
                0,
                0);
            vkCmdSetScissor(drawCmdBuffers[i], 0, 1, &scissor);

            vkCmdBindPipeline(drawCmdBuffers[i], VK_PIPELINE_BIND_POINT_GRAPHICS, pipelines.solidColor);

            for (auto& cube : cubes)
            {
                cube->draw(drawCmdBuffers[i], pipelineLayout);
            }

            vkCmdBindPipeline(drawCmdBuffers[i], VK_PIPELINE_BIND_POINT_GRAPHICS, pipelines.fire);

            fire->draw(drawCmdBuffers[i], pipelineLayout);

            vkCmdEndRenderPass(drawCmdBuffers[i]);

            err = vkEndCommandBuffer(drawCmdBuffers[i]);
            assert(!err);
        }
    }
    
    void buildBulletScene(){
        ///collision configuration contains default setup for memory, collision setup. Advanced users can create their own configuration.
        btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();

        ///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
        btCollisionDispatcher* dispatcher = new	btCollisionDispatcher(collisionConfiguration);

        ///btDbvtBroadphase is a good general purpose broadphase. You can also try out btAxis3Sweep.
        btBroadphaseInterface* overlappingPairCache = new btDbvtBroadphase();

        ///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
        btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;

       dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher,overlappingPairCache,solver,collisionConfiguration);

        dynamicsWorld->setGravity(btVector3(0,-10,0));

       for(int i=0;i<3 ;i++){
           dynamicsWorld->addRigidBody(cubes.at(i)->getRigidBody());
       }
       fire->addToWorld(dynamicsWorld);
    }

	void draw()
	{
		VkResult err;

		// Get next image in the swap chain (back/front buffer)
		err = swapChain.acquireNextImage(semaphores.presentComplete, &currentBuffer);
		assert(!err);

		submitPostPresentBarrier(swapChain.buffers[currentBuffer].image);

		// Command buffer to be sumitted to the queue
		submitInfo.commandBufferCount = 1;
		submitInfo.pCommandBuffers = &drawCmdBuffers[currentBuffer];

		// Submit to queue
		err = vkQueueSubmit(queue, 1, &submitInfo, VK_NULL_HANDLE);
		assert(!err);

		submitPrePresentBarrier(swapChain.buffers[currentBuffer].image);

		err = swapChain.queuePresent(queue, currentBuffer, semaphores.renderComplete);
		assert(!err);

		err = vkQueueWaitIdle(queue);
		assert(!err);
	}

	void prepareVertices()
	{
        cubes.push_back(new VulkanCube(device,this,glm::vec3(10.0,0.1,10.0),glm::vec3(0.1,0.1,0.1),glm::vec3(0,-0.6,0),0));

        cubes.push_back(new VulkanCube(device,this,glm::vec3(0.5,0.5,0.5),glm::vec3(1,0,0),glm::vec3(0,0,0),0));

        cubes.push_back(new VulkanCube(device,this,glm::vec3(0.5,0.5,0.5),glm::vec3(0,0,1),glm::vec3(0.5,2,0.5),1));

        addBurningPoints(cubes[1]->allo);

		// Binding description
		vertices.bindingDescriptions.resize(1);
		vertices.bindingDescriptions[0] =
			vkTools::initializers::vertexInputBindingDescription(
				VERTEX_BUFFER_BIND_ID,
				sizeof(Vertex),
				VK_VERTEX_INPUT_RATE_VERTEX);

		// Attribute descriptions
		// Describes memory layout and shader positions
		vertices.attributeDescriptions.resize(4);
		// Location 0 : Position
		vertices.attributeDescriptions[0] =
			vkTools::initializers::vertexInputAttributeDescription(
				VERTEX_BUFFER_BIND_ID,
				0,
				VK_FORMAT_R32G32B32_SFLOAT,
				0);
		// Location 1 : Color
		vertices.attributeDescriptions[1] =
			vkTools::initializers::vertexInputAttributeDescription(
				VERTEX_BUFFER_BIND_ID,
				1,
				VK_FORMAT_R32G32B32_SFLOAT,
				sizeof(float) * 3);
        // Location 2 : Texture coordinates
		vertices.attributeDescriptions[2] =
			vkTools::initializers::vertexInputAttributeDescription(
				VERTEX_BUFFER_BIND_ID,
				2,
				VK_FORMAT_R32G32_SFLOAT,
				sizeof(float) * 6);
        // Location 3 : Normal
		vertices.attributeDescriptions[3] =
			vkTools::initializers::vertexInputAttributeDescription(
				VERTEX_BUFFER_BIND_ID,
				3,
				VK_FORMAT_R32G32B32_SFLOAT,
				sizeof(float) * 8);

		vertices.inputState = vkTools::initializers::pipelineVertexInputStateCreateInfo();
		vertices.inputState.vertexBindingDescriptionCount = vertices.bindingDescriptions.size();
		vertices.inputState.pVertexBindingDescriptions = vertices.bindingDescriptions.data();
		vertices.inputState.vertexAttributeDescriptionCount = vertices.attributeDescriptions.size();
		vertices.inputState.pVertexAttributeDescriptions = vertices.attributeDescriptions.data();
	}

    void prepareParticles()
    {

        fire = new VulkanFire(device,this,glm::vec3(2,0,0));

        // Binding description
        particles.bindingDescriptions.resize(1);
        particles.bindingDescriptions[0] =
            vkTools::initializers::vertexInputBindingDescription(
                VERTEX_BUFFER_BIND_ID,
                sizeof(Particle),
                VK_VERTEX_INPUT_RATE_VERTEX);

        // Attribute descriptions
        // Describes memory layout and shader positions
        // Location 0 : Position
        particles.attributeDescriptions.push_back(
            vkTools::initializers::vertexInputAttributeDescription(
                VERTEX_BUFFER_BIND_ID,
                0,
                VK_FORMAT_R32G32B32A32_SFLOAT,
                0));
        // Location 1 : Color
        particles.attributeDescriptions.push_back(
            vkTools::initializers::vertexInputAttributeDescription(
                VERTEX_BUFFER_BIND_ID,
                1,
                VK_FORMAT_R32G32B32A32_SFLOAT,
                sizeof(float) * 4));
        // Location 2 : Alpha
        particles.attributeDescriptions.push_back(
            vkTools::initializers::vertexInputAttributeDescription(
                VERTEX_BUFFER_BIND_ID,
                2,
                VK_FORMAT_R32_SFLOAT,
                sizeof(float) * 8));
        // Location 3 : Size
        particles.attributeDescriptions.push_back(
            vkTools::initializers::vertexInputAttributeDescription(
                VERTEX_BUFFER_BIND_ID,
                3,
                VK_FORMAT_R32_SFLOAT,
                sizeof(float) * 9));
        // Location 4 : Rotation
        particles.attributeDescriptions.push_back(
            vkTools::initializers::vertexInputAttributeDescription(
                VERTEX_BUFFER_BIND_ID,
                4,
                VK_FORMAT_R32_SFLOAT,
                sizeof(float) * 10));
        // Location 5 : Type
        particles.attributeDescriptions.push_back(
            vkTools::initializers::vertexInputAttributeDescription(
                VERTEX_BUFFER_BIND_ID,
                5,
                VK_FORMAT_R32_SINT,
                sizeof(float) * 11));

        particles.inputState = vkTools::initializers::pipelineVertexInputStateCreateInfo();
        particles.inputState.vertexBindingDescriptionCount = particles.bindingDescriptions.size();
        particles.inputState.pVertexBindingDescriptions = particles.bindingDescriptions.data();
        particles.inputState.vertexAttributeDescriptionCount = particles.attributeDescriptions.size();
        particles.inputState.pVertexAttributeDescriptions = particles.attributeDescriptions.data();
    }

	void setupDescriptorPool()
	{
		// Example uses one ubo and one combined image sampler 
		std::vector<VkDescriptorPoolSize> poolSizes =
		{
            vkTools::initializers::descriptorPoolSize(VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, cubes.size()+1),
            vkTools::initializers::descriptorPoolSize(VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 10),
		};

		VkDescriptorPoolCreateInfo descriptorPoolInfo =
			vkTools::initializers::descriptorPoolCreateInfo(
				poolSizes.size(),
				poolSizes.data(),
                cubes.size()+1);

		VkResult vkRes = vkCreateDescriptorPool(device, &descriptorPoolInfo, nullptr, &descriptorPool);
		assert(!vkRes);
	}

	void setupDescriptorSetLayout()
	{
        std::vector<VkDescriptorSetLayoutBinding> setLayoutBindings =
        {
            // Binding 0 : Vertex shader uniform buffer
            vkTools::initializers::descriptorSetLayoutBinding(
            VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
                VK_SHADER_STAGE_VERTEX_BIT,
                0),
            // Binding 1 : Fragment shader image sampler
            vkTools::initializers::descriptorSetLayoutBinding(
                VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                VK_SHADER_STAGE_FRAGMENT_BIT,
                1),
            // Binding 2 : Fragment shader image sampler
            vkTools::initializers::descriptorSetLayoutBinding(
                VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                VK_SHADER_STAGE_FRAGMENT_BIT,
                2)
        };

		VkDescriptorSetLayoutCreateInfo descriptorLayout =
			vkTools::initializers::descriptorSetLayoutCreateInfo(
				setLayoutBindings.data(),
				setLayoutBindings.size());

		VkResult err = vkCreateDescriptorSetLayout(device, &descriptorLayout, nullptr, &descriptorSetLayout);
		assert(!err);

		VkPipelineLayoutCreateInfo pPipelineLayoutCreateInfo =
			vkTools::initializers::pipelineLayoutCreateInfo(
				&descriptorSetLayout,
				1);

		err = vkCreatePipelineLayout(device, &pPipelineLayoutCreateInfo, nullptr, &pipelineLayout);
		assert(!err);
	}

    void setupDescriptorSets()
    {
        for (auto& cube : cubes)
        {
            cube->setupDescriptorSet(descriptorPool, descriptorSetLayout);
        }
        fire->setupDescriptorSet(descriptorPool, descriptorSetLayout,particles.textures.sampler,particles.textures.fire,particles.textures.smoke);
    }

    void preparePipelinesCubes()
	{
		VkPipelineInputAssemblyStateCreateInfo inputAssemblyState =
			vkTools::initializers::pipelineInputAssemblyStateCreateInfo(
				VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST,
				0,
				VK_FALSE);

		VkPipelineRasterizationStateCreateInfo rasterizationState =
			vkTools::initializers::pipelineRasterizationStateCreateInfo(
				VK_POLYGON_MODE_FILL,
                VK_CULL_MODE_NONE,
				VK_FRONT_FACE_CLOCKWISE,
				0);

		VkPipelineColorBlendAttachmentState blendAttachmentState =
			vkTools::initializers::pipelineColorBlendAttachmentState(
				0xf,
				VK_FALSE);

		VkPipelineColorBlendStateCreateInfo colorBlendState =
			vkTools::initializers::pipelineColorBlendStateCreateInfo(
				1,
				&blendAttachmentState);

		VkPipelineDepthStencilStateCreateInfo depthStencilState =
			vkTools::initializers::pipelineDepthStencilStateCreateInfo(
				VK_TRUE,
				VK_TRUE,
				VK_COMPARE_OP_LESS_OR_EQUAL);

		VkPipelineViewportStateCreateInfo viewportState =
			vkTools::initializers::pipelineViewportStateCreateInfo(1, 1, 0);

		VkPipelineMultisampleStateCreateInfo multisampleState =
			vkTools::initializers::pipelineMultisampleStateCreateInfo(
				VK_SAMPLE_COUNT_1_BIT,
				0);

		std::vector<VkDynamicState> dynamicStateEnables = {
			VK_DYNAMIC_STATE_VIEWPORT,
			VK_DYNAMIC_STATE_SCISSOR,
			VK_DYNAMIC_STATE_LINE_WIDTH
		};
		VkPipelineDynamicStateCreateInfo dynamicState =
			vkTools::initializers::pipelineDynamicStateCreateInfo(
				dynamicStateEnables.data(),
				dynamicStateEnables.size(),
				0);

		// Color pipeline
		// Load shaders
		std::array<VkPipelineShaderStageCreateInfo, 2> shaderStages;

        shaderStages[0] = loadShader(getAssetPath() + "shaders/pipelines/base.vert.spv", VK_SHADER_STAGE_VERTEX_BIT);
		shaderStages[1] = loadShader(getAssetPath() + "shaders/pipelines/color.frag.spv", VK_SHADER_STAGE_FRAGMENT_BIT);
	
		VkGraphicsPipelineCreateInfo pipelineCreateInfo =
			vkTools::initializers::pipelineCreateInfo(
				pipelineLayout,
				renderPass,
				0);

		pipelineCreateInfo.pVertexInputState = &vertices.inputState;
		pipelineCreateInfo.pInputAssemblyState = &inputAssemblyState;
		pipelineCreateInfo.pRasterizationState = &rasterizationState;
		pipelineCreateInfo.pColorBlendState = &colorBlendState;
		pipelineCreateInfo.pMultisampleState = &multisampleState;
		pipelineCreateInfo.pViewportState = &viewportState;
		pipelineCreateInfo.pDepthStencilState = &depthStencilState;
		pipelineCreateInfo.pDynamicState = &dynamicState;
		pipelineCreateInfo.stageCount = shaderStages.size();
		pipelineCreateInfo.pStages = shaderStages.data();

		VkResult err = vkCreateGraphicsPipelines(device, pipelineCache, 1, &pipelineCreateInfo, nullptr, &pipelines.solidColor);
        assert(!err);
	}

    void preparePipelinesFire()
    {
        VkPipelineInputAssemblyStateCreateInfo inputAssemblyState =
            vkTools::initializers::pipelineInputAssemblyStateCreateInfo(
                VK_PRIMITIVE_TOPOLOGY_POINT_LIST,
                0,
                VK_FALSE);

        VkPipelineRasterizationStateCreateInfo rasterizationState =
            vkTools::initializers::pipelineRasterizationStateCreateInfo(
                VK_POLYGON_MODE_FILL,
                VK_CULL_MODE_BACK_BIT,
                VK_FRONT_FACE_CLOCKWISE,
                0);

        VkPipelineColorBlendAttachmentState blendAttachmentState =
            vkTools::initializers::pipelineColorBlendAttachmentState(
                0xf,
                VK_FALSE);

        VkPipelineColorBlendStateCreateInfo colorBlendState =
            vkTools::initializers::pipelineColorBlendStateCreateInfo(
                1,
                &blendAttachmentState);

        VkPipelineDepthStencilStateCreateInfo depthStencilState =
            vkTools::initializers::pipelineDepthStencilStateCreateInfo(
                VK_TRUE,
                VK_TRUE,
                VK_COMPARE_OP_LESS_OR_EQUAL);

        VkPipelineViewportStateCreateInfo viewportState =
            vkTools::initializers::pipelineViewportStateCreateInfo(1, 1, 0);

        VkPipelineMultisampleStateCreateInfo multisampleState =
            vkTools::initializers::pipelineMultisampleStateCreateInfo(
                VK_SAMPLE_COUNT_1_BIT,
                0);

        std::vector<VkDynamicState> dynamicStateEnables = {
            VK_DYNAMIC_STATE_VIEWPORT,
            VK_DYNAMIC_STATE_SCISSOR
        };
        VkPipelineDynamicStateCreateInfo dynamicState =
            vkTools::initializers::pipelineDynamicStateCreateInfo(
                dynamicStateEnables.data(),
                dynamicStateEnables.size(),
                0);

        // Load shaders
        std::array<VkPipelineShaderStageCreateInfo, 2> shaderStages;

        shaderStages[0] = loadShader(getAssetPath() + "shaders/particlefire/particle.vert.spv", VK_SHADER_STAGE_VERTEX_BIT);
        shaderStages[1] = loadShader(getAssetPath() + "shaders/particlefire/particle.frag.spv", VK_SHADER_STAGE_FRAGMENT_BIT);

        VkGraphicsPipelineCreateInfo pipelineCreateInfo =
            vkTools::initializers::pipelineCreateInfo(
                pipelineLayout,
                renderPass,
                0);

        pipelineCreateInfo.pVertexInputState = &particles.inputState;
        pipelineCreateInfo.pInputAssemblyState = &inputAssemblyState;
        pipelineCreateInfo.pRasterizationState = &rasterizationState;
        pipelineCreateInfo.pColorBlendState = &colorBlendState;
        pipelineCreateInfo.pMultisampleState = &multisampleState;
        pipelineCreateInfo.pViewportState = &viewportState;
        pipelineCreateInfo.pDepthStencilState = &depthStencilState;
        pipelineCreateInfo.pDynamicState = &dynamicState;
        pipelineCreateInfo.stageCount = shaderStages.size();
        pipelineCreateInfo.pStages = shaderStages.data();

        depthStencilState.depthWriteEnable = VK_FALSE;

        // Premulitplied alpha
        blendAttachmentState.blendEnable = VK_TRUE;
        blendAttachmentState.srcColorBlendFactor = VK_BLEND_FACTOR_ONE;
        blendAttachmentState.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
        blendAttachmentState.colorBlendOp = VK_BLEND_OP_ADD;
        blendAttachmentState.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
        blendAttachmentState.dstAlphaBlendFactor = VK_BLEND_FACTOR_ZERO;
        blendAttachmentState.alphaBlendOp = VK_BLEND_OP_ADD;
        blendAttachmentState.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;

        VkResult err = vkCreateGraphicsPipelines(device, pipelineCache, 1, &pipelineCreateInfo, nullptr, &pipelines.fire);
        assert(!err);
    }

	void updateUniformBuffers()
	{
        glm::mat4 proj = glm::perspective(glm::radians(60.0f), (float)(width) / (float)height, 0.1f, 256.0f);
        glm::mat4 view = glm::lookAt(glm::vec3(0.0f, 0.0f, -zoom),glm::vec3(0,0,0),glm::vec3(0,1,0));
        //view = glm::lookAtLH(glm::vec3(0.0f, 0.0f, zoom),glm::vec3(0,0,0),glm::vec3(0,-1,0));
        view = glm::scale(view,glm::vec3(1,-1,1));
        //uboVS.modelMatrix = glm::mat4();
        view = glm::rotate(view, glm::radians(rotation.x), glm::vec3(1.0f, 0.0f, 0.0f));
        view = glm::rotate(view, glm::radians(rotation.y), glm::vec3(0.0f, 1.0f, 0.0f));
        view = glm::rotate(view, glm::radians(rotation.z), glm::vec3(0.0f, 0.0f, 1.0f));
        for (auto& cube : cubes)
        {
            cube->updateProjView(proj,view);
        }
        fire->updateProjView(proj,view);
	}




	void prepare()
	{
		VulkanExampleBase::prepare();
		loadTextures();
		prepareVertices();
        prepareParticles();
		setupDescriptorSetLayout();
        preparePipelinesCubes();
        preparePipelinesFire();
		setupDescriptorPool();
        setupDescriptorSets();
		buildCommandBuffers();
        buildBulletScene();

        glm::mat4 proj = glm::perspective(glm::radians(60.0f), (float)width / (float)height, 0.1f, 256.0f);
        glm::mat4 view = glm::lookAt(glm::vec3(0.0f, 0.0f, 5),glm::vec3(0,0,0),glm::vec3(0,1,0));
        view = glm::scale(view,glm::vec3(1,-1,1));

        for (auto& cube : cubes)
        {
            cube->updateProjView(proj,view);
        }
        fire->updateProjView(proj,view);
		prepared = true;
	}

	virtual void render()
    {
        if (!prepared)
            return;
        dynamicsWorld->stepSimulation(frameTimer);
		vkDeviceWaitIdle(device);
		draw();
		vkDeviceWaitIdle(device);
	}

	virtual void viewChanged()
	{
        updateUniformBuffers();
	}

    virtual void pick(float x, float y){

        float fw = (float)width;
        float fh = (float)height;
        float mx = (x)/(fw);

        glm::vec4 lRayStart_NDC(
            (mx  - 0.5f) * 2.0f,
            (2.0f * y)/ fh -1.0f, // [0, 768] -> [-1,1]
            -1.0, // The near plane maps to Z=-1 in Normalized Device Coordinates
            1.0f
        );

        glm::vec4 ori(
            0.0f,
            0.0f,
            0.0f,
            1.0f
        );

        glm::mat4 proj = glm::perspective(glm::radians(60.0f), (float)(width) / (float)height, 0.1f, 256.0f);
        glm::mat4 view = glm::lookAt(glm::vec3(0.0f, 0.0f, -zoom),glm::vec3(0,0,0),glm::vec3(0,1,0));
        //view = glm::lookAtLH(glm::vec3(0.0f, 0.0f, zoom),glm::vec3(0,0,0),glm::vec3(0,-1,0));
        view = glm::scale(view,glm::vec3(1,-1,1));
        //uboVS.modelMatrix = glm::mat4();
        view = glm::rotate(view, glm::radians(rotation.x), glm::vec3(1.0f, 0.0f, 0.0f));
        view = glm::rotate(view, glm::radians(rotation.y), glm::vec3(0.0f, 1.0f, 0.0f));
        view = glm::rotate(view, glm::radians(rotation.z), glm::vec3(0.0f, 0.0f, 1.0f));


        // Faster way (just one inverse)
        glm::mat4 M = glm::inverse(proj * view);
        glm::vec4 lRayStart_world = M * lRayStart_NDC; lRayStart_world/=lRayStart_world.w;

        ori = glm::inverse(view)*ori; ori/=ori.w;

        glm::vec4 out_end = ori + (lRayStart_world-ori)*1000.0f;

        btCollisionWorld::ClosestRayResultCallback RayCallback(
            btVector3(ori.x, ori.y, ori.z),
            btVector3(out_end.x, out_end.y, out_end.z)
        );
        dynamicsWorld->rayTest(
            btVector3(ori.x, ori.y, ori.z),
            btVector3(out_end.x, out_end.y, out_end.z),
            RayCallback
        );
        if(RayCallback.hasHit()) {
            std::cout << "hit called "<< RayCallback.m_hitPointWorld.x() << " " << RayCallback.m_hitPointWorld.y() << " " << RayCallback.m_hitPointWorld.z()<< std::endl;
            //trans.setOrigin(RayCallback.m_hitPointWorld);

            //selectBody->setWorldTransform(trans);

        }
        else{
            //trans.setOrigin(btVector3(0,0,0));
        }
    }


    void addBurningPoints(std::vector<glm::vec3> data){

        int size = data.size();


        for(int i=0;i<size;i+=4){

            testt(data,i);
        }

        }
                void testt(std::vector<glm::vec3> data,int i){
                    Fade_2D dt;
                    glm::mat3 mat,matt;
                    glm::vec3 u,v,w,a;
                    std::vector<Point2> vPoints;
                    std::vector<Segment2> vSegments1;
                    std::vector<Triangle2*> vTriangles2;
                    std::vector<ConstraintGraph2*> vCG;

                    std::stringstream sstm;
            u = glm::normalize(data.at(i+1)-data.at(i));
            w = data.at(i+3);
            v = glm::cross(u,w);

            mat = glm::mat3();

            mat[0]=u;
            mat[1]=v;
            mat[2]=w;

            std::cout <<"u " << u.x << " " << u.y << " " <<u.z << std::endl;
            std::cout <<"v " << v.x << " " << v.y << " " <<v.z << std::endl ;
            std::cout <<"w " << w.x << " " << w.y << " " <<w.z << std::endl << std::endl;

            matt =glm::transpose(mat);
            for(int j=0;j<3;j++){
                a = matt*data.at(i+j);
                std::cout <<"point " << data.at(i+j).x << " " << data.at(i+j).y << " " <<data.at(i+j).z << std::endl;
                std::cout <<"point " << a.x << " " << a.y << " " <<a.z << std::endl << std::endl;

                vPoints.push_back(Point2(a.x,a.y));
            }
            dt.insert(vPoints);
            std::cout <<"pointf " << std::endl;
            for(int j=0;j<3;j++){
                Point2& p0(vPoints[j]);
                Point2& p1(vPoints[(j+1)%3]);
                vSegments1.push_back(Segment2(p0,p1));
            }

            ConstraintGraph2* pCG1=dt.createConstraint(vSegments1,CIS_CONSTRAINED_DELAUNAY);

            vCG.push_back(pCG1);

            Zone2* pZone=dt.createZone(vCG,ZL_GROW,vPoints[0]);
            dt.applyConstraintsAndZones();
            dt.refine(pZone,27,0.01,0.1,true);

            pZone->getTriangles(vTriangles2);

            sstm << "allo" << i << ".ps";
            Visualizer2 vis2(sstm.str());
            for(std::vector<Triangle2*>::iterator it(vTriangles2.begin());it!=vTriangles2.end();++it)
            {
                vis2.addObject(**it,Color(1,0,0,0.01,true));
            }
            vis2.writeFile();

            dt.deleteZone(pZone);
            sstm.str("");
            sstm.clear();

            vPoints.clear();
            vSegments1.clear();
            vCG.clear();
        }
};
VulkanExample *vulkanExample;

#if defined(_WIN32)
LRESULT CALLBACK WndProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
	if (vulkanExample != NULL)
	{
		vulkanExample->handleMessages(hWnd, uMsg, wParam, lParam);
	}
	return (DefWindowProc(hWnd, uMsg, wParam, lParam));
}
#elif defined(__linux__) && !defined(__ANDROID__)
static void handleEvent(const xcb_generic_event_t *event)
{
	if (vulkanExample != NULL)
	{
		vulkanExample->handleEvent(event);
	}
}
#endif

// Main entry point
#if defined(_WIN32)
// Windows entry point
int APIENTRY WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR pCmdLine, int nCmdShow)
#elif defined(__ANDROID__)
// Android entry point
void android_main(android_app* state)
#elif defined(__linux__)
// Linux entry point
int main(const int argc, const char *argv[])
#endif
{
#if defined(__ANDROID__)
	// Removing this may cause the compiler to omit the main entry point 
	// which would make the application crash at start
	app_dummy();
#endif
	vulkanExample = new VulkanExample();
#if defined(_WIN32)
	vulkanExample->setupWindow(hInstance, WndProc);
#elif defined(__ANDROID__)
	// Attach vulkan example to global android application state
	state->userData = vulkanExample;
	state->onAppCmd = VulkanExample::handleAppCommand;
	state->onInputEvent = VulkanExample::handleAppInput;
	vulkanExample->androidApp = state;
#elif defined(__linux__)
	vulkanExample->setupWindow();
#endif
#if !defined(__ANDROID__)
	vulkanExample->initSwapchain();
	vulkanExample->prepare();
#endif
	vulkanExample->renderLoop();
	delete(vulkanExample);
#if !defined(__ANDROID__)
	return 0;
#endif
}
