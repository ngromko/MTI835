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
#define ENABLE_VALIDATION true

class VulkanExample: public VulkanExampleBase 
{
private:
	vkTools::VulkanTexture textureColorMap;
    std::chrono::time_point<std::chrono::high_resolution_clock> tStart;
    btRigidBody* selectBody;
    std::vector<uint32_t> objectData;

    float time = 0;

public:
	struct {
		VkPipelineVertexInputStateCreateInfo inputState;
		std::vector<VkVertexInputBindingDescription> bindingDescriptions;
		std::vector<VkVertexInputAttributeDescription> attributeDescriptions;
	} vertices;

    struct {
        vkTools::VulkanTexture smoke;
        vkTools::VulkanTexture fire;
        // We use a custom sampler to change some sampler
        // attributes required for rotation the uv coordinates
        // inside the shader for alpha blended textures
        VkSampler sampler;
    } textures;

    btDiscreteDynamicsWorld* dynamicsWorld;

    vkTools::UniformData objectsUniforme;

    std::vector<VulkanCube*> cubes;
    std::vector<VulkanCube*> burningCubes;
    VulkanFire* fire;
    int stop=0;
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

        vkDestroySampler(device, textures.sampler, nullptr);
		
		vkDestroyPipelineLayout(device, pipelineLayout, nullptr);
        vkDestroyDescriptorSetLayout(device, descriptorSetLayout, nullptr);

		textureLoader->destroyTexture(textureColorMap);
        textureLoader->destroyTexture(textures.smoke);
        textureLoader->destroyTexture(textures.fire);

        vkUnmapMemory(device, objectsUniforme.memory);
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
            &textures.smoke);
        textureLoader->loadTexture(
            getAssetPath() + "textures/particle_fire.ktx",
            VK_FORMAT_BC3_UNORM_BLOCK,
            &textures.fire);

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
        samplerCreateInfo.maxLod = textures.fire.mipLevels;
        // Enable anisotropic filtering
        samplerCreateInfo.maxAnisotropy = 8;
        samplerCreateInfo.anisotropyEnable = VK_TRUE;
        // Use a different border color (than the normal texture loader) for additive blending
        samplerCreateInfo.borderColor = VK_BORDER_COLOR_FLOAT_TRANSPARENT_BLACK;
        VkResult err = vkCreateSampler(device, &samplerCreateInfo, nullptr, &textures.sampler);
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

            fire->compute(drawCmdBuffers[i]);

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
            for (auto& cube : burningCubes)
            {
                cube->draw(drawCmdBuffers[i], pipelineLayout);
            }

            fire->draw(drawCmdBuffers[i]);

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

        dynamicsWorld->setGravity(btVector3(0,-2,0));

        for (auto& cube : cubes)
        {
            dynamicsWorld->addRigidBody(cube->getRigidBody());
        }
        for(auto& cube : burningCubes){
           dynamicsWorld->addRigidBody(cube->getRigidBody());
        }
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
        objectData.push_back(0);
        fire=new VulkanFire(device,this);

        std::vector<glm::vec3> allo;
        //cubes.push_back(new VulkanCube(device,this,glm::vec3(10.0,0.1,10.0),glm::vec3(0.1,0.1,0.1),glm::vec3(5,-0.1,5),0));
        /*int objnumber=0;
        for(int i = 0;i<5;i++){
            for(int j = 0;j<5;j++){
                for(int k = 1;k<3;k++){
                    burningCubes.push_back(new VulkanCube(device,this,glm::vec3(0.5f,0.5f,0.5f),glm::vec3(1,0,0),glm::vec3(0.5+i*2,0.5+k*2,0.5+j*2),0,allo));

                    objectData.push_back(fire->addBurningPoints(allo,objnumber++));
                }
            }
        }*/

        burningCubes.push_back(new VulkanCube(device,this,glm::vec3(1.0f,1.5f,1.0f),glm::vec3(1,0,0),glm::vec3(6.5,1.5f,5),0,allo));

        objectData.push_back(fire->addBurningPoints(allo,0));

        burningCubes.push_back(new VulkanCube(device,this,glm::vec3(0.5,0.5,0.5),glm::vec3(0,0,1),glm::vec3(5,0.5f,5),0,allo));

        objectData.push_back(fire->addBurningPoints(allo,1));

        burningCubes.push_back(new VulkanCube(device,this,glm::vec3(0.5,0.5,0.5),glm::vec3(0,0,1),glm::vec3(6.5,4.5f,5),0,allo));

        objectData.push_back(fire->addBurningPoints(allo,2));

        std::cout <<" errrrre "<< allo.size() << std::endl;

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

	void setupDescriptorPool()
	{
		// Example uses one ubo and one combined image sampler 
		std::vector<VkDescriptorPoolSize> poolSizes =
		{
            vkTools::initializers::descriptorPoolSize(VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 60),
            vkTools::initializers::descriptorPoolSize(VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 10),
            vkTools::initializers::descriptorPoolSize(VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 100),
		};

		VkDescriptorPoolCreateInfo descriptorPoolInfo =
			vkTools::initializers::descriptorPoolCreateInfo(
				poolSizes.size(),
				poolSizes.data(),
                100);

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
        std::cout<<"enter"<<std::endl;

        VkDeviceSize  size = 16*4*burningCubes.size();

        std::cout<<size<<std::endl;
        createBuffer(
            VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
            VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT,
            size,
            nullptr,
            &objectsUniforme.buffer,
            &objectsUniforme.memory,
            &objectsUniforme.descriptor);

        fire->init(queue,cmdPool,renderPass,descriptorPool, &objectsUniforme.descriptor, textures.sampler,textures.fire,textures.smoke,getAssetPath());
        uint32_t offset =0;
        uint8_t* pData;

        VkResult err = vkMapMemory(device, objectsUniforme.memory, 0, size, 0, (void **)&pData);
        assert(!err);
        std::cout<<"test"<<std::endl;
        for (auto& cube : cubes)
        {
            cube->setupDescriptorSet(descriptorPool, descriptorSetLayout,offset ,pData);
        }
        for (auto& cube : burningCubes)
        {
            std::cout<<"test"<<std::endl;
            cube->setupDescriptorSet(descriptorPool, descriptorSetLayout,offset ,pData);
            offset+=16*4;
        }

        VkCommandBuffer moveBurnCmd;

        VkCommandBufferAllocateInfo cmdBufAllocateInfo =
            vkTools::initializers::commandBufferAllocateInfo(
                cmdPool,
                VK_COMMAND_BUFFER_LEVEL_PRIMARY,
                1);

        VK_CHECK_RESULT(vkAllocateCommandBuffers(device, &cmdBufAllocateInfo, &moveBurnCmd));

        fire->buildMoveBurnCommand(moveBurnCmd);

        for (auto& cube : burningCubes)
        {
            cube->setupBurnCommand(queue,moveBurnCmd);
        }

        glm::mat4 model4 = glm::mat4();

        model4 = glm::translate(model4,glm::vec3(6.5,4.5f,5));
        //view = glm::rotate(view, glm::radians(rotation.x), glm::vec3(1.0f, 0.0f, 0.0f));
        model4 = glm::rotate(model4,glm::radians(45.0f),glm::vec3(1,0,0));
        model4 = glm::rotate(model4,glm::radians(45.0f),glm::vec3(0,1,0));

        burningCubes[2]->updateModel(model4);


        btMatrix3x3 bob;
        btQuaternion nob = btQuaternion(btVector3(1,0,0),glm::radians(45.0f));
        nob += btQuaternion(btVector3(0,1,0),glm::radians(45.0f));
        burningCubes[2]->getRigidBody()->getWorldTransform().setRotation(nob);

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

        view = glm::translate(view,glm::vec3(-5.0f,0.0f,-5.0f));
        for (auto& cube : cubes)
        {
            cube->updateProjView(proj,view);
        }
        for (auto& cube : burningCubes)
        {
            cube->updateProjView(proj,view);
        }
        fire->updateProjView(proj,view);
	}

	void prepare()
	{
		VulkanExampleBase::prepare();
        loadTextures();std::cout<<"bumbo"<<std::endl;
        prepareVertices();std::cout<<"bumbo1"<<std::endl;
        setupDescriptorSetLayout();std::cout<<"bumbo2"<<std::endl;
        preparePipelinesCubes();std::cout<<"bumbo3"<<std::endl;
        setupDescriptorPool();std::cout<<"bumbo4"<<std::endl;
        setupDescriptorSets();std::cout<<"bumbo5"<<std::endl;
        buildCommandBuffers();std::cout<<"bumbo6"<<std::endl;
        buildBulletScene();std::cout<<"bumbo7"<<std::endl;

        updateUniformBuffers();std::cout<<"bumbo8"<<std::endl;
		prepared = true;
	}

	virtual void render()
    {
        if (!prepared)
            return;
        dynamicsWorld->stepSimulation(frameTimer);
        fire->updateTime(frameTimer);
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

        view = glm::translate(view,glm::vec3(-5.0f,0.0f,-5.0f));

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
            fire->cliked(queue,glm::vec4( RayCallback.m_hitPointWorld.x(), RayCallback.m_hitPointWorld.y(), RayCallback.m_hitPointWorld.z(),0.0f));
        }
        else{
            //trans.setOrigin(btVector3(0,0,0));
        }
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
