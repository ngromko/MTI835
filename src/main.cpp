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
#include "vulkanObject.h"
#include "vulkancube.h"
#include "VulkanMesh.h"
#include "particlefire.h"
#include "Shadow.h"

#define VERTEX_BUFFER_BIND_ID 0
#define ENABLE_VALIDATION false

class VulkanExample: public VulkanExampleBase 
{
private:
    vkTools::VulkanTexture paper;
    vkTools::VulkanTexture wood;
    vkTools::VulkanTexture metal;
    vkTools::VulkanTexture burn;
    vkTools::VulkanTexture armorTexture;
    vkMeshLoader::MeshBuffer quad;

    struct {
        vkTools::UniformData scene;
        vkTools::UniformData objectsUniforme;
    } uniformData;

    btVector3 m_pickPos;
    btScalar m_pickDist;
    btPoint2PointConstraint* m_pickedConstraint;
    btRigidBody* m_pickedBody;

    vkTools::UniformData bPointsStorageBuffer;
    vkTools::UniformData lightsStorageBuffer;

    std::vector<BurningPoint> bPoints;

    std::chrono::time_point<std::chrono::high_resolution_clock> tStart;
    btRigidBody* selectBody;
    float time = 0;

    //Matrice de projection et de vue pour la scene
    struct{
        glm::mat4 projection;
        glm::mat4 view;
        float AR;
    } ubo;

    std::vector<glm::vec4> lights;

    struct Light{
       glm::vec4 pos;
    };

    //Permet d'obtenir les coordonnées monde de la souris
    glm::vec4 getRayTo(float x, float y){
        float fw = (float)width;
        float fh = (float)height;
        float mx = (x)/(fw);

        glm::vec4 lRayStart_NDC(
            (mx  - 0.5f) * 2.0f,
            (2.0f * y)/ fh -1.0f, // [0, 768] -> [-1,1]
            -1.0, // The near plane maps to Z=-1 in Normalized Device Coordinates
            1.0f
        );

        // Faster way (just one inverse)
        glm::mat4 M = glm::inverse(ubo.projection * ubo.view);
        glm::vec4 lRayStart_world = M * lRayStart_NDC;
        lRayStart_world/=lRayStart_world.w;
        return lRayStart_world;
    }
public:
	struct {
		VkPipelineVertexInputStateCreateInfo inputState;
		std::vector<VkVertexInputBindingDescription> bindingDescriptions;
		std::vector<VkVertexInputAttributeDescription> attributeDescriptions;
	} vertices;

    //Textures et sampler pour les particules de feu
    struct {
        vkTools::VulkanTexture smoke;
        vkTools::VulkanTexture fire;
        // We use a custom sampler to change some sampler
        // attributes required for rotation the uv coordinates
        // inside the shader for alpha blended textures
        VkSampler sampler;
    } textures;

    std::vector<vkMeshLoader::VertexLayout> vertexLayout =
    {
        vkMeshLoader::VERTEX_LAYOUT_POSITION,
        vkMeshLoader::VERTEX_LAYOUT_UV,
        vkMeshLoader::VERTEX_LAYOUT_NORMAL
    };

    //Monde de simulation de Bullet
    btDiscreteDynamicsWorld* dynamicsWorld;

    std::vector<VulkanObject*> papers;
    std::vector<VulkanObject*> woods;
    std::vector<VulkanObject*> metals;
    std::vector<VulkanObject*> allObjects;

    VulkanFire* fire;
    VkCommandBuffer computeCommand;
    //VkCommandBuffer deferred;
    VkSubmitInfo computeSubmitInfo;
    int stop=0;

    struct {
        VkPipelineLayout scene;
        VkPipelineLayout deferred;
    } pipelineLayouts;

    struct {
        VkDescriptorSet scene;
        VkDescriptorSet paper;
        VkDescriptorSet wood;
        VkDescriptorSet metal;
    } descriptorSets;

	VkDescriptorSetLayout descriptorSetLayout;
    VkDescriptorSetLayout descriptorSetLayoutMeterial;


    struct {
        VkPipeline scene;
        VkPipeline deferred;
        VkPipeline blend;
    } pipelines;

    struct{
        SceneMaterial papier = {4.0f,30.0f,1.0f};
        SceneMaterial bois = {15.0f,75.0f,5.0f};
        SceneMaterial metal = {-1.0f,100.0f,10.0f};
        SceneMaterial papierStatic = {-4.0f,30.0f,0.0f};
        SceneMaterial boisStatic = {15.0f,75.0f,0.0f};
        SceneMaterial metalStatic = {-1.0f,100.0f,0.0f};
    } materiaux;

    FrameBuffer deferredFb;

    struct PConst {
        uint32_t index;
        float factor;
    }pCostant;

    Shadow* shadow;

    VkSemaphore offscreenSemaphore = VK_NULL_HANDLE;
    VkSemaphore computeSemaphore = VK_NULL_HANDLE;

	VulkanExample() : VulkanExampleBase(ENABLE_VALIDATION)
	{
        //Caractéristique initiale de la caméra
        zoom = -15.0f;
        rotation = glm::vec3(30.0f, 0.0f, 0.0f);
		title = "Vulkan Example - Using pipelines";

        for(int i=0;i<5;i++){
            float x= -5.0f + 25.0f*(rand() / double(RAND_MAX));
            float z= -5.0f + 25.0f*(rand() / double(RAND_MAX));

            lights.push_back(glm::vec4(x,10.0f,z,1.0f));
        }
        lights.push_back(glm::vec4(15.0f,5.0f,5.0f,1.0f));
        lights.push_back(glm::vec4(5.0f,5.0f,15.0f,1.0f));

        pCostant.factor = 1.0f/lights.size();
	}

	~VulkanExample()
	{
		// Clean up used Vulkan resources 
		// Note : Inherited destructor cleans up resources stored in base class
        //vkDestroyPipeline(device, pipelines.wireFrame, nullptr);
        //vkDestroyPipeline(device, pipelines.texture, nullptr);

        vkDestroySampler(device, textures.sampler, nullptr);
		
        //vkDestroyRenderPass(device, FrameBuf.renderPass, nullptr);

        // Pipelibes
        vkDestroyPipeline(device, pipelines.scene, nullptr);
        //vkDestroyPipeline(device, pipelines., nullptr);

        vkDestroyPipelineLayout(device, pipelineLayouts.scene, nullptr);
        //vkDestroyPipelineLayout(device, pipelineLayouts., nullptr);

        vkDestroyDescriptorSetLayout(device, descriptorSetLayout, nullptr);
        vkDestroyDescriptorSetLayout(device, descriptorSetLayoutMeterial, nullptr);

        textureLoader->destroyTexture(wood);
        textureLoader->destroyTexture(armorTexture);
        textureLoader->destroyTexture(paper);
        textureLoader->destroyTexture(burn);
        textureLoader->destroyTexture(textures.smoke);
        textureLoader->destroyTexture(textures.fire);

        //deferredFb.FreeResources(device);
        //vkMeshLoader::freeMeshBufferResources(device, &quad);

        vkUnmapMemory(device, uniformData.objectsUniforme.memory);

        vkTools::destroyUniformData(device, &uniformData.scene);
        vkTools::destroyUniformData(device, &uniformData.objectsUniforme);

        vkDestroySemaphore(device, offscreenSemaphore, nullptr);
        vkDestroySemaphore(device, computeSemaphore, nullptr);

	}

    //La fonction textureLoader->loadTexture vient du tutoriel. Parmet de charger une texture dans le GPU
    void loadTextures()
    {
        textureLoader->loadTexture(
            getAssetPath() + "textures/crate_bc3.ktx",
            VK_FORMAT_BC3_UNORM_BLOCK,
            &wood);

        textureLoader->loadTexture(
            getAssetPath() + "textures/darkmetal_bc3.ktx",
            VK_FORMAT_BC3_UNORM_BLOCK,
            &metal);

        // Particles
        textureLoader->loadTexture(
            getAssetPath() + "textures/particle_smoke.ktx",
            VK_FORMAT_BC3_UNORM_BLOCK,
            &textures.smoke);
        textureLoader->loadTexture(
            getAssetPath() + "textures/particle_fire.ktx",
            VK_FORMAT_BC3_UNORM_BLOCK,
            &textures.fire);

        textureLoader->loadTexture(
                    getAssetPath() + "models/armor/colormap.ktx",
            VK_FORMAT_BC3_UNORM_BLOCK,
            &armorTexture);

        textureLoader->loadTexture(
                    getAssetPath() + "textures/paper.dds",
            VK_FORMAT_BC3_UNORM_BLOCK,
            &paper);


        textureLoader->loadTexture(
            getAssetPath() + "textures/burned.dds",
            VK_FORMAT_BC3_UNORM_BLOCK,
            &burn);

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

    //Création des commandes pour le rendu et compute shader.
    //C'est ce qui se retrouverait dans la boucle while 1 deOpenGL
    void buildCommandBuffers()
    {

        computeCommand = createCommandBuffer(VK_COMMAND_BUFFER_LEVEL_PRIMARY,true);

        //Exécution des compute shaders
        fire->compute(computeCommand);

        vkEndCommandBuffer(computeCommand);

        computeSubmitInfo = vkTools::initializers::submitInfo();
        //computeSubmitInfo.pCommandBuffers = &computeCommand;
        // Create a semaphore used to synchronize offscreen rendering and usage
        VkSemaphoreCreateInfo semaphoreCreateInfo = vkTools::initializers::semaphoreCreateInfo();
        VK_CHECK_RESULT(vkCreateSemaphore(device, &semaphoreCreateInfo, nullptr, &offscreenSemaphore));
        VK_CHECK_RESULT(vkCreateSemaphore(device, &semaphoreCreateInfo, nullptr, &computeSemaphore));

        VkCommandBufferBeginInfo cmdBufInfo = vkTools::initializers::commandBufferBeginInfo();

        VkClearValue clearValues[2];
        clearValues[0].color = defaultClearColor;
        clearValues[1].depthStencil = { 1.0f, 0 };

        VkRenderPassBeginInfo renderPassBeginInfo = vkTools::initializers::renderPassBeginInfo();
        renderPassBeginInfo.renderArea.offset.x = 0;
        renderPassBeginInfo.renderArea.offset.y = 0;
        renderPassBeginInfo.renderArea.extent.width = width;
        renderPassBeginInfo.renderArea.extent.height = height;

        VkDescriptorSet sets[2] =  {descriptorSets.scene,descriptorSets.paper};

        VkResult err;

        //Il y a deux commandes une pour chaque image du swapchain.
        for (int32_t i = 0; i < drawCmdBuffers.size(); ++i)
        {

            VkViewport viewport = vkTools::initializers::viewport(
                (float)width,
                (float)height,
                0.0f,
                1.0f);

            VkRect2D scissor = vkTools::initializers::rect2D(
                width,
                height,
                0,
                0);

            // Set target frame buffer
            renderPassBeginInfo.framebuffer = frameBuffers[i];

            err = vkBeginCommandBuffer(drawCmdBuffers[i], &cmdBufInfo);
            assert(!err);

            //Construction du shadow cube map pour la première lumière
            shadow->buildOffscreenCommandBuffer(drawCmdBuffers[i],allObjects,&bPointsStorageBuffer.buffer,0);

            //Nouvelle image alors le framebuffer est effacé (equivalent de GL_CLEAR)
            renderPassBeginInfo.clearValueCount = 2;
            renderPassBeginInfo.pClearValues = clearValues;
            renderPassBeginInfo.renderPass = renderPass;

            vkCmdBeginRenderPass(drawCmdBuffers[i], &renderPassBeginInfo, VK_SUBPASS_CONTENTS_INLINE);


            vkCmdSetViewport(drawCmdBuffers[i], 0, 1, &viewport);
            vkCmdSetScissor(drawCmdBuffers[i], 0, 1, &scissor);
            vkCmdBindPipeline(drawCmdBuffers[i], VK_PIPELINE_BIND_POINT_GRAPHICS, pipelines.scene);

            pCostant.index=0;

            //L'index de la lumière en cours est transféré au fragment shader.
            vkCmdPushConstants(
                drawCmdBuffers[i],
                pipelineLayouts.scene,
                VK_SHADER_STAGE_FRAGMENT_BIT,
                0,
                8,
                &pCostant);

            //La texture de papier est binder pour dessiner les objets en papier
            sets[1] = descriptorSets.paper;
            vkCmdBindDescriptorSets(drawCmdBuffers[i], VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayouts.scene, 0, 2,sets , 0, NULL);

            for (auto& cube : papers)
            {
                cube->draw(drawCmdBuffers[i], &bPointsStorageBuffer.buffer);
            }

            //La texture de bois est binder pour dessiner les objets en bois
            sets[1] = descriptorSets.wood;
            vkCmdBindDescriptorSets(drawCmdBuffers[i], VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayouts.scene, 0, 2,sets , 0, NULL);

            for (auto& cube : woods)
            {
                cube->draw(drawCmdBuffers[i], &bPointsStorageBuffer.buffer);
            }

            //La texture de papier est binder pour dessiner les objets en métal
            sets[1] = descriptorSets.metal;
            vkCmdBindDescriptorSets(drawCmdBuffers[i], VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayouts.scene, 0, 2,sets , 0, NULL);

            for (auto& cube : metals)
            {
                cube->draw(drawCmdBuffers[i], &bPointsStorageBuffer.buffer);
            }

            //Pour les prochaine lumière on enléve le clear parce qu'on veut additionner les valeurs de couleur. Nouveau renderpass dans lequel il n'y a pas de clear.
            renderPassBeginInfo.clearValueCount = 0;
            renderPassBeginInfo.pClearValues = VK_NULL_HANDLE;
            renderPassBeginInfo.renderPass = renderPassBlend;


            for(uint32_t j=1;j<lights.size();j++){

                vkCmdEndRenderPass(drawCmdBuffers[i]);

                shadow->buildOffscreenCommandBuffer(drawCmdBuffers[i],allObjects,&bPointsStorageBuffer.buffer,j);

                vkCmdBeginRenderPass(drawCmdBuffers[i], &renderPassBeginInfo, VK_SUBPASS_CONTENTS_INLINE);

                vkCmdSetViewport(drawCmdBuffers[i], 0, 1, &viewport);
                vkCmdSetScissor(drawCmdBuffers[i], 0, 1, &scissor);
                //Utilisation d'un pipeline qui va additionner la couleur calculée avec la couleur déjà en place dans le buffer.
                vkCmdBindPipeline(drawCmdBuffers[i], VK_PIPELINE_BIND_POINT_GRAPHICS, pipelines.blend);

                pCostant.index=j;

                vkCmdPushConstants(
                    drawCmdBuffers[i],
                    pipelineLayouts.scene,
                    VK_SHADER_STAGE_FRAGMENT_BIT,
                    0,
                    8,
                    &pCostant);

                sets[1] = descriptorSets.paper;
                vkCmdBindDescriptorSets(drawCmdBuffers[i], VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayouts.scene, 0, 2,sets , 0, NULL);

                for (auto& cube : papers)
                {
                    cube->draw(drawCmdBuffers[i], &bPointsStorageBuffer.buffer);
                }

                sets[1] = descriptorSets.wood;
                vkCmdBindDescriptorSets(drawCmdBuffers[i], VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayouts.scene, 0, 2,sets , 0, NULL);

                for (auto& cube : woods)
                {
                    cube->draw(drawCmdBuffers[i], &bPointsStorageBuffer.buffer);
                }

                sets[1] = descriptorSets.metal;
                vkCmdBindDescriptorSets(drawCmdBuffers[i], VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayouts.scene, 0, 2,sets , 0, NULL);

                for (auto& cube : metals)
                {
                    cube->draw(drawCmdBuffers[i], &bPointsStorageBuffer.buffer);
                }
            }
            //Les particules sont dessinées
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

        //Ajout de plans pour borner la simulation.
        btCollisionShape* plan1 = new btStaticPlaneShape(btVector3(-1,0,0),-10);
        btRigidBody::btRigidBodyConstructionInfo
                        groundRigidBodyCI1(0, NULL, plan1, btVector3(0, 0, 0));

        btRigidBody* rp1 = new btRigidBody(groundRigidBodyCI1);

        btCollisionShape* plan2 = new btStaticPlaneShape(btVector3(1,0,0),0);
        btRigidBody::btRigidBodyConstructionInfo
                        groundRigidBodyCI2(0, NULL, plan2, btVector3(0, 0, 0));

          btRigidBody* rp2 = new btRigidBody(groundRigidBodyCI2);

        btCollisionShape* plan3 = new btStaticPlaneShape(btVector3(0,0,-1),-10);
        btRigidBody::btRigidBodyConstructionInfo
                        groundRigidBodyCI3(0, NULL, plan3, btVector3(0, 0, 0));

          btRigidBody* rp3 = new btRigidBody(groundRigidBodyCI3);

        btCollisionShape* plan4 = new btStaticPlaneShape(btVector3(0,0,1),0);
        btRigidBody::btRigidBodyConstructionInfo
                        groundRigidBodyCI4(0, NULL, plan4, btVector3(0, 0, 0));

        btRigidBody* rp4 = new btRigidBody(groundRigidBodyCI4);

        dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher,overlappingPairCache,solver,collisionConfiguration);

        dynamicsWorld->setGravity(btVector3(0,-2,0));
        dynamicsWorld->addRigidBody(rp1);
        dynamicsWorld->addRigidBody(rp2);
        dynamicsWorld->addRigidBody(rp3);
        dynamicsWorld->addRigidBody(rp4);

        for (auto& cube : allObjects)
        {
            dynamicsWorld->addRigidBody(cube->getRigidBody());
        }
    }

	void draw()
	{
        //Vient du tutoriel.
        VulkanExampleBase::prepareFrame();

        //Shadow Map
        // Wait for swap chain presentation to finish
        submitInfo.pWaitSemaphores = &semaphores.presentComplete;
        submitInfo.pSignalSemaphores = &computeSemaphore;

        // Signal ready with offscreen semaphore
        submitInfo.commandBufferCount = 1;
        // Calcul compute shader
        submitInfo.pCommandBuffers = &computeCommand;

        // Submit to queue
        VK_CHECK_RESULT(vkQueueSubmit(queue, 1, &submitInfo, VK_NULL_HANDLE));


        /*// Submit work
        submitInfo.pWaitSemaphores = &computeSemaphore;
        submitInfo.pSignalSemaphores = &offscreenSemaphore;
        submitInfo.pCommandBuffers = shadow->getCommandBuffer();
        VK_CHECK_RESULT(vkQueueSubmit(queue, 1, &submitInfo, VK_NULL_HANDLE));*/

        // Scene rendering

        // Wait for offscreen semaphore
        submitInfo.pWaitSemaphores = &computeSemaphore;
        // Signal ready with render complete semaphpre
        submitInfo.pSignalSemaphores = &semaphores.renderComplete;

        // Submit work
        submitInfo.pCommandBuffers = &drawCmdBuffers[currentBuffer];
        VK_CHECK_RESULT(vkQueueSubmit(queue, 1, &submitInfo, VK_NULL_HANDLE));

        //Vient du tutoriel. Affiche le frame buffer
        VulkanExampleBase::submitFrame();
	}

	void prepareVertices()
	{
        fire=new VulkanFire(device,this);
        glm::mat4 cubeModel;

        uint32_t objectNumber=0;
        cubeModel = glm::translate(glm::mat4(),glm::vec3(5,-1,5));
        metals.push_back(new VulkanCube(device,this,queue,glm::vec3(6.0,1,6.0),cubeModel,materiaux.metalStatic,objectNumber++,bPoints));
        cubeModel = glm::translate(glm::mat4(),glm::vec3(-1,5,5));
        metals.push_back(new VulkanCube(device,this,queue,glm::vec3(1,6.0,6.0),cubeModel,materiaux.metalStatic,objectNumber++,bPoints));
        cubeModel = glm::translate(glm::mat4(),glm::vec3(5,5,-1));
        metals.push_back(new VulkanCube(device,this,queue,glm::vec3(6.0,6.0,1.0),cubeModel,materiaux.metalStatic,objectNumber++,bPoints));

        cubeModel = glm::translate(glm::mat4(),glm::vec3(8.0f,2.0f,8.0f));
        papers.push_back(new VulkanCube(device,this,queue,glm::vec3(0.5f,0.5f,0.5f),cubeModel,materiaux.papier,objectNumber++,bPoints));
        cubeModel = glm::translate(glm::mat4(),glm::vec3(2.0f,6.0f,2.0f));
        papers.push_back(loadBurningMesh(getAssetPath() + "models/armor/cylinder.obj", glm::vec3(0.10f,-0.10f,0.10f), cubeModel, materiaux.papier, objectNumber++));

        cubeModel = glm::translate(glm::mat4(),glm::vec3(5.0f,2.0f,15.0f));
        woods.push_back(new VulkanCube(device,this,queue,glm::vec3(1.0f,1.0f,1.0f),cubeModel,materiaux.bois,objectNumber++,bPoints));
        cubeModel = glm::translate(glm::mat4(),glm::vec3(5.0f,6.0f,5.0f));
        woods.push_back(loadBurningMesh(getAssetPath() + "models/armor/torus.obj", glm::vec3(0.10f,-0.10f,0.10f), cubeModel, materiaux.bois, objectNumber++));

        allObjects.insert(allObjects.end(),metals.begin(),metals.end());
        allObjects.insert(allObjects.end(),papers.begin(),papers.end());
        allObjects.insert(allObjects.end(),woods.begin(),woods.end());
    }

    //Envois du tableau de lumière dans le GPU
    void setupLights(){
        uint32_t storageBufferSize = lights.size() * sizeof(glm::vec4);

        // Staging
        // SSBO is static, copy to device local memory
        // This results in better performance

        struct {
            VkDeviceMemory memory;
            VkBuffer buffer;
        } stagingBuffer;
        createBuffer(
            VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
            VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT,
            storageBufferSize,
            lights.data(),
            &stagingBuffer.buffer,
            &stagingBuffer.memory);

        createBuffer(
            VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
            storageBufferSize,
            nullptr,
            &lightsStorageBuffer.buffer,
            &lightsStorageBuffer.memory);

        // Copy to staging buffer
        VkCommandBuffer copyCmd = createCommandBuffer(VK_COMMAND_BUFFER_LEVEL_PRIMARY, true);

        VkBufferCopy copyRegion = {};
        copyRegion.size = storageBufferSize;
        vkCmdCopyBuffer(
            copyCmd,
            stagingBuffer.buffer,
            lightsStorageBuffer.buffer,
            1,
            &copyRegion);
        flushCommandBuffer(copyCmd, queue, true);
        vkFreeMemory(device, stagingBuffer.memory, nullptr);
        vkDestroyBuffer(device, stagingBuffer.buffer, nullptr);
        lightsStorageBuffer.descriptor.range = storageBufferSize;
        lightsStorageBuffer.descriptor.buffer = lightsStorageBuffer.buffer;
        lightsStorageBuffer.descriptor.offset = 0;
    }

    //Les poits émetteurs sont envoyés sur le GPU et les éléments de ces derniers qui seront utilisés dans
    //le vertex shader sont décrits
    void prepareBurningPoints(){
        // Setup and fill the compute shader storage buffers for
        // vertex positions and velocities

        uint32_t storageBufferSize = bPoints.size() * sizeof(BurningPoint);

        // Staging
        // SSBO is static, copy to device local memory
        // This results in better performance

        struct {
            VkDeviceMemory memory;
            VkBuffer buffer;
        } stagingBuffer;
        createBuffer(
            VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
            VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT,
            storageBufferSize,
            bPoints.data(),
            &stagingBuffer.buffer,
            &stagingBuffer.memory);

        createBuffer(
            VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
            storageBufferSize,
            nullptr,
            &bPointsStorageBuffer.buffer,
            &bPointsStorageBuffer.memory);
        // Copy to staging buffer
        VkCommandBuffer copyCmd = createCommandBuffer(VK_COMMAND_BUFFER_LEVEL_PRIMARY, true);

        VkBufferCopy copyRegion = {};
        copyRegion.size = storageBufferSize;
        vkCmdCopyBuffer(
            copyCmd,
            stagingBuffer.buffer,
            bPointsStorageBuffer.buffer,
            1,
            &copyRegion);
        flushCommandBuffer(copyCmd, queue, true);
        vkFreeMemory(device, stagingBuffer.memory, nullptr);
        vkDestroyBuffer(device, stagingBuffer.buffer, nullptr);
        bPointsStorageBuffer.descriptor.range = storageBufferSize;
        bPointsStorageBuffer.descriptor.buffer = bPointsStorageBuffer.buffer;
        bPointsStorageBuffer.descriptor.offset = 0;
        // Binding description
        vertices.bindingDescriptions.resize(1);
        vertices.bindingDescriptions[0] =
            vkTools::initializers::vertexInputBindingDescription(
                VERTEX_BUFFER_BIND_ID,
                sizeof(BurningPoint),
                VK_VERTEX_INPUT_RATE_VERTEX);

        // Attribute descriptions
        // Describes memory layout and shader positions
        vertices.attributeDescriptions.resize(5);
        // Location 0 : Position
        vertices.attributeDescriptions[0] =
            vkTools::initializers::vertexInputAttributeDescription(
                VERTEX_BUFFER_BIND_ID,
                0,
                VK_FORMAT_R32G32B32_SFLOAT,
                0);
        // Location 1 : Normal
        vertices.attributeDescriptions[1] =
            vkTools::initializers::vertexInputAttributeDescription(
                VERTEX_BUFFER_BIND_ID,
                1,
                VK_FORMAT_R32G32B32A32_SFLOAT,
                sizeof(float) * 8);
        // Location 2 : Texture coordinates
        vertices.attributeDescriptions[2] =
            vkTools::initializers::vertexInputAttributeDescription(
                VERTEX_BUFFER_BIND_ID,
                2,
                VK_FORMAT_R32G32_SFLOAT,
                sizeof(float) * 16);
        // Location 3 : Heat
        vertices.attributeDescriptions[3] =
            vkTools::initializers::vertexInputAttributeDescription(
                VERTEX_BUFFER_BIND_ID,
                3,
                VK_FORMAT_R32_SFLOAT,
                sizeof(float) * 31);
        // Location 4 : MaxHeat
        vertices.attributeDescriptions[4] =
            vkTools::initializers::vertexInputAttributeDescription(
                VERTEX_BUFFER_BIND_ID,
                4,
                VK_FORMAT_R32_SFLOAT,
                sizeof(float) * 19);

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
            vkTools::initializers::descriptorPoolSize(VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 100),
            vkTools::initializers::descriptorPoolSize(VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 10),
            vkTools::initializers::descriptorPoolSize(VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 150),
		};

		VkDescriptorPoolCreateInfo descriptorPoolInfo =
			vkTools::initializers::descriptorPoolCreateInfo(
				poolSizes.size(),
				poolSizes.data(),
                200);

		VkResult vkRes = vkCreateDescriptorPool(device, &descriptorPoolInfo, nullptr, &descriptorPool);
		assert(!vkRes);
	}

    //DescriptorLayout décrit quel type de données seront Binder au shaders.
	void setupDescriptorSetLayout()
	{

        std::vector<VkDescriptorSetLayoutBinding> setLayoutBindings =
        {
            // Binding 0 : Vertex shader uniform buffer
            vkTools::initializers::descriptorSetLayoutBinding(
                    VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
                        VK_SHADER_STAGE_VERTEX_BIT,
                        0),
            // Binding 2 : Fragment shader image sampler burned
            vkTools::initializers::descriptorSetLayoutBinding(
                VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                VK_SHADER_STAGE_FRAGMENT_BIT,
                2),
            // Binding 3 : Shadow maps
            vkTools::initializers::descriptorSetLayoutBinding(
                VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                VK_SHADER_STAGE_FRAGMENT_BIT,
                3),
            // Binding 4 : Lights
            vkTools::initializers::descriptorSetLayoutBinding(
                VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                VK_SHADER_STAGE_FRAGMENT_BIT,
                4)
        };

		VkDescriptorSetLayoutCreateInfo descriptorLayout =
			vkTools::initializers::descriptorSetLayoutCreateInfo(
				setLayoutBindings.data(),
				setLayoutBindings.size());

		VkResult err = vkCreateDescriptorSetLayout(device, &descriptorLayout, nullptr, &descriptorSetLayout);
		assert(!err);

        // Binding 1 : Fragment shader image sampler normal texture// Binding 0 : Vertex shader uniform buffer
        VkDescriptorSetLayoutBinding matLayoutBinding =
                vkTools::initializers::descriptorSetLayoutBinding(
                    VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                    VK_SHADER_STAGE_FRAGMENT_BIT,
                    1);

        descriptorLayout =
            vkTools::initializers::descriptorSetLayoutCreateInfo(
                &matLayoutBinding,
                1);

        err = vkCreateDescriptorSetLayout(device, &descriptorLayout, nullptr, &descriptorSetLayoutMeterial);
        assert(!err);

        VkDescriptorSetLayout layouts[2] = {descriptorSetLayout, descriptorSetLayoutMeterial};

		VkPipelineLayoutCreateInfo pPipelineLayoutCreateInfo =
			vkTools::initializers::pipelineLayoutCreateInfo(
                layouts,
                2);

        // Push constants for cube map face view matrices
            VkPushConstantRange pushConstantRange =
                vkTools::initializers::pushConstantRange(
                    VK_SHADER_STAGE_FRAGMENT_BIT,
                    8,
                    0);

            // Push constant ranges are part of the pipeline layout
            pPipelineLayoutCreateInfo.pushConstantRangeCount = 1;
            pPipelineLayoutCreateInfo.pPushConstantRanges = &pushConstantRange;

        err = vkCreatePipelineLayout(device, &pPipelineLayoutCreateInfo, nullptr, &pipelineLayouts.scene);
		assert(!err);

	}

    //Les descriptorSet disent quelles données seront associés aux types définis dans le ou les descriptorLayout.
    void setupDescriptorSets()
    {

        VkDeviceSize  size = 16*4*allObjects.size();
        if(size>0){
            //Réserve un espace mémoire sur le GPU pour les matrices modèles
            createBuffer(
                VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
                VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT,
                size,
                nullptr,
                &uniformData.objectsUniforme.buffer,
                &uniformData.objectsUniforme.memory,
                &uniformData.objectsUniforme.descriptor);

            uint32_t offset =0;
            uint8_t* pData;

            VkResult err = vkMapMemory(device, uniformData.objectsUniforme.memory, 0, size, 0, (void **)&pData);
            assert(!err);

            // Color map image descriptor
            VkDescriptorImageInfo texDescriptorPaper =
                vkTools::initializers::descriptorImageInfo(
                    paper.sampler,
                    paper.view,
                    VK_IMAGE_LAYOUT_GENERAL);

            // Color map image descriptor
            VkDescriptorImageInfo texDescriptorWood =
                vkTools::initializers::descriptorImageInfo(
                    wood.sampler,
                    wood.view,
                    VK_IMAGE_LAYOUT_GENERAL);

            // Color map image descriptor
            VkDescriptorImageInfo texDescriptorMetal =
                vkTools::initializers::descriptorImageInfo(
                    metal.sampler,
                    metal.view,
                    VK_IMAGE_LAYOUT_GENERAL);

            // Burn image descriptor
            VkDescriptorImageInfo texDescriptorBurned =
                vkTools::initializers::descriptorImageInfo(
                    burn.sampler,
                    burn.view,
                    VK_IMAGE_LAYOUT_GENERAL);

            // Burn image descriptor
            VkDescriptorImageInfo texDescriptorShadow =
                vkTools::initializers::descriptorImageInfo(
                    shadow->getCubeMapTexture()->sampler,
                    shadow->getCubeMapTexture()->view,
                    VK_IMAGE_LAYOUT_GENERAL);

            VkDescriptorSetAllocateInfo allocInfo =
                vkTools::initializers::descriptorSetAllocateInfo(
                    descriptorPool,
                    &descriptorSetLayoutMeterial,
                    1);

            VkResult vkRes = vkAllocateDescriptorSets(device, &allocInfo, &descriptorSets.paper);
            assert(!vkRes);

            vkRes = vkAllocateDescriptorSets(device, &allocInfo, &descriptorSets.wood);
            assert(!vkRes);

            vkRes = vkAllocateDescriptorSets(device, &allocInfo, &descriptorSets.metal);
            assert(!vkRes);

            allocInfo =
                vkTools::initializers::descriptorSetAllocateInfo(
                    descriptorPool,
                    &descriptorSetLayout,
                    1);

            vkRes = vkAllocateDescriptorSets(device, &allocInfo, &descriptorSets.scene);
            assert(!vkRes);

            std::vector<VkWriteDescriptorSet> writeDescriptorSets =
            {
                // Binding 1 : Texture papier
                vkTools::initializers::writeDescriptorSet(
                    descriptorSets.paper,
                    VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                    1,
                    &texDescriptorPaper),
                // Binding 1 : Texture bois
                vkTools::initializers::writeDescriptorSet(
                    descriptorSets.wood,
                    VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                    1,
                    &texDescriptorWood),
                // Binding 1 : Texture metal
                vkTools::initializers::writeDescriptorSet(
                    descriptorSets.metal,
                    VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                    1,
                    &texDescriptorMetal),
                // Binding 0 : Vertex shader uniform buffer
                vkTools::initializers::writeDescriptorSet(
                    descriptorSets.scene,
                    VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
                    0,
                    &uniformData.scene.descriptor),
                // Binding 2 : Fragment shader image sampler
                vkTools::initializers::writeDescriptorSet(
                    descriptorSets.scene,
                    VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                    2,
                    &texDescriptorBurned),
                // Binding 3 : Fragment shader image sampler
                vkTools::initializers::writeDescriptorSet(
                    descriptorSets.scene,
                    VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                    3,
                    &texDescriptorShadow),
                // Binding 4 : Lights
                vkTools::initializers::writeDescriptorSet(
                    descriptorSets.scene,
                    VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                    4,
                    &lightsStorageBuffer.descriptor)
            };


            vkUpdateDescriptorSets(device, 3, writeDescriptorSets.data(), 0, NULL);

            vkUpdateDescriptorSets(device, writeDescriptorSets.size(), writeDescriptorSets.data(), 0, NULL);

            //Dis aux objets quelle partie de l'espace mémoire de matrices modèles ils doivent transférer leur matrice modèle.
            for (auto& cube : metals)
            {
                cube->setupMemory(offset ,pData);
                offset+=16*4;
            }
            for (auto& cube : papers)
            {
                cube->setupMemory(offset ,pData);
                offset+=16*4;
            }
            for (auto& cube : woods)
            {
                cube->setupMemory(offset ,pData);
                offset+=16*4;
            }
        }

        VkDescriptorBufferInfo infos[3] = {bPointsStorageBuffer.descriptor,uniformData.objectsUniforme.descriptor,uniformData.scene.descriptor};
        fire->init(queue,cmdPool,renderPass,descriptorPool, infos,bPoints.size(), textures.sampler,textures.fire,textures.smoke,getAssetPath());
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
            VK_DYNAMIC_STATE_SCISSOR,
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
        shaderStages[1] = loadShader(getAssetPath() + "shaders/pipelines/texture.frag.spv", VK_SHADER_STAGE_FRAGMENT_BIT);
        VkGraphicsPipelineCreateInfo pipelineCreateInfo =
            vkTools::initializers::pipelineCreateInfo(
                pipelineLayouts.scene,
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

        VkResult err = vkCreateGraphicsPipelines(device, pipelineCache, 1, &pipelineCreateInfo, nullptr, &pipelines.scene);
        assert(!err);

        // Additive blending
        blendAttachmentState.colorWriteMask = 0xF;
        blendAttachmentState.blendEnable = VK_TRUE;
        blendAttachmentState.colorBlendOp = VK_BLEND_OP_ADD;
        blendAttachmentState.srcColorBlendFactor = VK_BLEND_FACTOR_ONE;
        blendAttachmentState.dstColorBlendFactor = VK_BLEND_FACTOR_ONE;
        blendAttachmentState.alphaBlendOp = VK_BLEND_OP_MAX;
        blendAttachmentState.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
        blendAttachmentState.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE;

        vkCreateGraphicsPipelines(device, pipelineCache, 1, &pipelineCreateInfo, nullptr, &pipelines.blend);
    }

    void prepareUniformBuffer()
    {
        createBuffer(
           VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
           sizeof(ubo),
           &ubo,
           &uniformData.scene.buffer,
           &uniformData.scene.memory,
           &uniformData.scene.descriptor);
    }

	void updateUniformBuffers()
	{
        ubo.projection = glm::perspective(glm::radians(60.0f), (float)(width) / (float)height, 0.1f, 256.0f);
        //ubo.projection = glm::perspective((float)(M_PI / 2.0), 1.0f, 0.1f,1024.0f);

        glm::mat4 view = glm::lookAt(glm::vec3(0.0f, 0.0f, -zoom),glm::vec3(0,0,0),glm::vec3(0,1,0));
        //view = glm::lookAtLH(glm::vec3(0.0f, 0.0f, zoom),glm::vec3(0,0,0),glm::vec3(0,-1,0));
        view = glm::scale(view,glm::vec3(1,-1,1));

        //uboVS.modelMatrix = glm::mat4();
        view = glm::rotate(view, glm::radians(rotation.x), glm::vec3(1.0f, 0.0f, 0.0f));
        view = glm::rotate(view, glm::radians(rotation.y), glm::vec3(0.0f, 1.0f, 0.0f));
        view = glm::rotate(view, glm::radians(rotation.z), glm::vec3(0.0f, 0.0f, 1.0f));

        //view = glm::scale(glm::mat4(),glm::vec3(1,-1,1));
        //view = glm::rotate(view, glm::radians(90.0f), glm::vec3(0.0f, 1.0f, 0.0f));
        //ubo.view = glm::translate(view,glm::vec3(-5.0f,zoom,-5.0f));
        ubo.view = glm::translate(view,glm::vec3(-5.0f,0.0f,-5.0f));
        ubo.AR = (float)(width) / (float)height;

        uint8_t *pData;
        VkResult err = vkMapMemory(device, uniformData.scene.memory, 0, sizeof(ubo), 0, (void **)&pData);
        assert(!err);
        memcpy(pData, &ubo, sizeof(ubo));
        vkUnmapMemory(device, uniformData.scene.memory);
    }

	void prepare()
	{
        VkFormat fbDepthFormat;
        // Find a suitable depth format
        VkBool32 validDepthFormat = vkTools::getSupportedDepthFormat(physicalDevice, &fbDepthFormat);
        assert(validDepthFormat);

		VulkanExampleBase::prepare();
        loadTextures();
        prepareVertices();
        prepareBurningPoints();
        prepareUniformBuffer();
        setupDescriptorSetLayout();
        preparePipelinesCubes();
        setupDescriptorPool();
        setupLights();
        shadow = new Shadow(device,queue,&lightsStorageBuffer.descriptor,fbDepthFormat,this,&vertices.inputState);
        setupDescriptorSets();
        buildCommandBuffers();
        buildBulletScene();

        updateUniformBuffers();
		prepared = true;
	}

	virtual void render()
    {

        if (!prepared || stop>2)
            return;
        dynamicsWorld->stepSimulation(frameTimer);
        fire->updateTime(frameTimer);
		vkDeviceWaitIdle(device);
        draw();
		vkDeviceWaitIdle(device);
       //stop++;
	}

	virtual void viewChanged()
	{
        updateUniformBuffers();
	}

    virtual void pick(float x, float y,bool grab){

        glm::vec4 m_rayTo = getRayTo(x,y);

        glm::vec4 m_rayFrom(
            0.0f,
            0.0f,
            0.0f,
            1.0f
        );

        m_rayFrom = glm::inverse(ubo.view)*m_rayFrom; m_rayFrom/=m_rayFrom.w;

        m_rayTo = m_rayFrom + (m_rayTo-m_rayFrom)*1000.0f;

        btVector3 btRayTo = btVector3(m_rayTo.x, m_rayTo.y, m_rayTo.z);
        btVector3 btRayFrom = btVector3(m_rayFrom.x, m_rayFrom.y, m_rayFrom.z);

        btCollisionWorld::ClosestRayResultCallback rayCallback(
            btRayFrom,
            btRayTo
        );
        dynamicsWorld->rayTest(
            btRayFrom,
            btRayTo,
            rayCallback
        );
        if(rayCallback.hasHit()) {
            m_pickPos = rayCallback.m_hitPointWorld;
            if(grab){
                btRigidBody* pBody = btRigidBody::upcast(rayCallback.m_collisionObject);
                if (pBody)
                {
                    // Code for adding a constraint from Bullet Demo's DemoApplication.cpp
                    if (!(pBody->isStaticObject() || pBody->isKinematicObject()))
                    {
                        m_pickedBody = pBody;
                        m_pickedBody->setActivationState(DISABLE_DEACTIVATION);
                        //printf("pickPos=%f,%f,%f\n",pickPos.getX(),pickPos.getY(),pickPos.getZ());
                        btVector3 localPivot = pBody->getCenterOfMassTransform().inverse() * m_pickPos;
                        btPoint2PointConstraint* m_pickConstraint = new btPoint2PointConstraint(*pBody, localPivot);
                        dynamicsWorld->addConstraint(m_pickConstraint, true);
                        m_pickedConstraint = m_pickConstraint;
                        btScalar mousePickClamping = 30.f;
                        m_pickConstraint->m_setting.m_impulseClamp = mousePickClamping;
                        //very weak constraint for picking
                        m_pickConstraint->m_setting.m_tau = 0.001f;

                    }
                }
            }
            else{
                fire->cliked(queue,glm::vec4( m_pickPos.x(), m_pickPos.y(), m_pickPos.z(),0.0f));
            }
            m_pickDist = (m_pickPos - btRayFrom).length();
        }
    }

    virtual void drag(float x, float y){
        glm::vec4 m_rayTo = getRayTo(x,y);

        glm::vec4 m_rayFrom(
            0.0f,
            0.0f,
            0.0f,
            1.0f
        );

        m_rayFrom = glm::inverse(ubo.view)*m_rayFrom; m_rayFrom/=m_rayFrom.w;

        btVector3 btRayTo = btVector3(m_rayTo.x, m_rayTo.y, m_rayTo.z);
        btVector3 btRayFrom = btVector3(m_rayFrom.x, m_rayFrom.y, m_rayFrom.z);

        btPoint2PointConstraint* pickCon = static_cast<btPoint2PointConstraint*>(m_pickedConstraint);
        if (pickCon)
        {
            //keep it at the same picking distance

            btVector3 dir = btRayTo-btRayFrom;
            dir.normalize();
            dir *= m_pickDist;

            btVector3 newPivotB = btRayFrom + dir;
            pickCon->setPivotB(newPivotB);
        }
    }
    virtual void releaseGrab(){
        if (m_pickedConstraint && dynamicsWorld)
        {
            dynamicsWorld->removeConstraint(m_pickedConstraint);
            delete m_pickedConstraint;
            m_pickedConstraint = NULL;
            m_pickedBody->setDeactivationTime( 0.f );
            m_pickedBody = NULL;
        }
    }

    VulkanObject* loadBurningMesh(std::string filename,glm::vec3 scale,glm::mat4 model, SceneMaterial mater, uint32_t objectNumber){
        VulkanMeshLoader *mesh = new VulkanMeshLoader();

        //mesh->LoadMesh est une fonction du tutoriel
        mesh->LoadMesh(filename);
        int burnStart=0;
        BurningPoint bp;

        for(int i=0;i<mesh->m_Entries.size();i++){
            burnStart = bPoints.size();
            if(mater.life<=0.0f){
                for (int j = mesh->m_Entries[i].Indices.size()-1;j>=0; j--)
                {
                    bp.pos = glm::vec4(mesh->m_Entries[i].Vertices[mesh->m_Entries[i].Indices[j]].m_pos*scale,0.0f);
                    bp.basePos = glm::vec4(mesh->m_Entries[i].Vertices[mesh->m_Entries[i].Indices[j]].m_pos*scale,1.0f);
                    bp.normal= bp.baseNorm = glm::vec4(mesh->m_Entries[i].Vertices[mesh->m_Entries[i].Indices[j]].m_normal*glm::vec3(1.0f,-1.0f,1.0f),0.0f);
                    bp.uvCoord = mesh->m_Entries[i].Vertices[mesh->m_Entries[i].Indices[j]].m_tex;
                    bp.nCount = 0;
                    bPoints.push_back(bp);

                }
                return new VulkanMesh(device,this,queue,mesh->m_Entries[i].Indices,materiaux.papier,model,objectNumber,burnStart,bPoints);
            }
            else{

                std::vector<Vertex> vBuffer;
                vBuffer.clear();
                vBuffer.resize(mesh->m_Entries[i].Indices.size());
                for (int j = 0; j<mesh->m_Entries[i].Indices.size(); j++)
                {
                    vBuffer[j].pos = mesh->m_Entries[i].Vertices[mesh->m_Entries[i].Indices[j]].m_pos*scale;
                    vBuffer[j].uv = mesh->m_Entries[i].Vertices[mesh->m_Entries[i].Indices[j]].m_tex;
                    vBuffer[j].normal = mesh->m_Entries[i].Vertices[mesh->m_Entries[i].Indices[j]].m_normal;
                }
                return new VulkanMesh(device,this,queue,vBuffer,materiaux.papier,model,objectNumber,burnStart,bPoints);
            }
        }
    }

    //Pas utilisé
    void createAttachment(VkFormat format, VkImageUsageFlagBits usage, FrameBufferAttachment *attachment, VkCommandBuffer layoutCmd, bool depthSample = false)
    {
        VkImageAspectFlags aspectMask = 0;
        VkImageLayout imageLayout;

        attachment->format = format;

        if (usage & VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT)
        {
            aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
            imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        }
        if (usage & VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT)
        {
            aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT | VK_IMAGE_ASPECT_STENCIL_BIT;
            imageLayout = depthSample ? VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL : VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
            //attachment->isDepth = true;
        }

        assert(aspectMask > 0);

        VkImageCreateInfo image = vkTools::initializers::imageCreateInfo();
        image.imageType = VK_IMAGE_TYPE_2D;
        image.format = format;
        image.extent.width = deferredFb.width;
        image.extent.height = deferredFb.height;
        image.extent.depth = 1;
        image.mipLevels = 1;
        image.arrayLayers = 1;
        image.samples = VK_SAMPLE_COUNT_1_BIT;
        image.tiling = VK_IMAGE_TILING_OPTIMAL;
        image.usage = usage | VK_IMAGE_USAGE_SAMPLED_BIT;

        VkMemoryAllocateInfo memAlloc = vkTools::initializers::memoryAllocateInfo();
        VkMemoryRequirements memReqs;

        VK_CHECK_RESULT(vkCreateImage(device, &image, nullptr, &attachment->image));
        vkGetImageMemoryRequirements(device, attachment->image, &memReqs);
        memAlloc.allocationSize = memReqs.size;
        memAlloc.memoryTypeIndex = getMemoryType(memReqs.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
        VK_CHECK_RESULT(vkAllocateMemory(device, &memAlloc, nullptr, &attachment->mem));
        VK_CHECK_RESULT(vkBindImageMemory(device, attachment->image, attachment->mem, 0));

        if (usage & VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT)
        {
            // Set the initial layout to shader read instead of attachment
            // This is done as the render loop does the actualy image layout transitions
            vkTools::setImageLayout(
                layoutCmd,
                attachment->image,
                aspectMask,
                VK_IMAGE_LAYOUT_UNDEFINED,
                VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
        }
        else
        {
            vkTools::setImageLayout(
                layoutCmd,
                attachment->image,
                aspectMask,
                VK_IMAGE_LAYOUT_UNDEFINED,
                imageLayout);
        }

        VkImageViewCreateInfo imageView = vkTools::initializers::imageViewCreateInfo();
        imageView.viewType = VK_IMAGE_VIEW_TYPE_2D;
        imageView.format = format;
        imageView.subresourceRange = {};
        imageView.subresourceRange.aspectMask = aspectMask;
        imageView.subresourceRange.baseMipLevel = 0;
        imageView.subresourceRange.levelCount = 1;
        imageView.subresourceRange.baseArrayLayer = 0;
        imageView.subresourceRange.layerCount = 1;
        imageView.image = attachment->image;
        VK_CHECK_RESULT(vkCreateImageView(device, &imageView, nullptr, &attachment->view));
    }

    //Pas utilisé
    // Prepare the framebuffer for offscreen rendering with multiple attachments used as render targets inside the fragment shaders
    void deferredSetup()
    {
        VkCommandBuffer layoutCmd = VulkanExampleBase::createCommandBuffer(VK_COMMAND_BUFFER_LEVEL_PRIMARY, true);

        deferredFb.width = FB_DIM;
        deferredFb.height = FB_DIM;

        // Four attachments (3 color, 1 depth)
        deferredFb.attachments.resize(4);

        // Color attachments
        // Attachment 0: (World space) Positions
        createAttachment(
            VK_FORMAT_R16G16B16A16_SFLOAT,
            VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT,
            &deferredFb.attachments[0],
            layoutCmd);

        // Attachment 1: (World space) Normals
        createAttachment(
            VK_FORMAT_R16G16B16A16_SFLOAT,
            VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT,
            &deferredFb.attachments[1],
            layoutCmd);

        // Attachment 1: Albedo (color)
        createAttachment(
            VK_FORMAT_R8G8B8A8_UNORM,
            VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT,
            &deferredFb.attachments[2],
            layoutCmd);

        // Depth attachment
        // Find a suitable depth format
        VkFormat attDepthFormat;
        VkBool32 validDepthFormat = vkTools::getSupportedDepthFormat(physicalDevice, &attDepthFormat);
        assert(validDepthFormat);

        createAttachment(
            attDepthFormat,
            VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT,
            &deferredFb.attachments[3],
            layoutCmd);

        VulkanExampleBase::flushCommandBuffer(layoutCmd, queue, true);

        // Set up separate renderpass with references
        // to the color and depth attachments

        std::array<VkAttachmentDescription, 4> attachmentDescs = {};

        // Init attachment properties
        for (uint32_t i = 0; i < 4; ++i)
        {
            attachmentDescs[i].samples = VK_SAMPLE_COUNT_1_BIT;
            attachmentDescs[i].loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
            attachmentDescs[i].storeOp = VK_ATTACHMENT_STORE_OP_STORE;
            attachmentDescs[i].stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
            attachmentDescs[i].stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
            attachmentDescs[i].format = deferredFb.attachments[i].format;
            if (i == 3)
            {
                attachmentDescs[i].initialLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
                attachmentDescs[i].finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
            }
            else
            {
                attachmentDescs[i].initialLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
                attachmentDescs[i].finalLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
            }
        }

        std::vector<VkAttachmentReference> colorReferences;
        colorReferences.push_back({ 0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL });
        colorReferences.push_back({ 1, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL });
        colorReferences.push_back({ 2, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL });

        VkAttachmentReference depthReference = {};
        depthReference.attachment = 3;
        depthReference.layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

        VkSubpassDescription subpass = {};
        subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
        subpass.pColorAttachments = colorReferences.data();
        subpass.colorAttachmentCount = static_cast<uint32_t>(colorReferences.size());
        subpass.pDepthStencilAttachment = &depthReference;

        VkRenderPassCreateInfo renderPassInfo = {};
        renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
        renderPassInfo.pAttachments = attachmentDescs.data();
        renderPassInfo.attachmentCount = static_cast<uint32_t>(attachmentDescs.size());
        renderPassInfo.subpassCount = 1;
        renderPassInfo.pSubpasses = &subpass;
        VK_CHECK_RESULT(vkCreateRenderPass(device, &renderPassInfo, nullptr, &deferredFb.renderPass));

        std::vector<VkImageView> attachments;
        for (auto attachment : deferredFb.attachments)
        {
            attachments.push_back(attachment.view);
        }

        VkFramebufferCreateInfo fbufCreateInfo = {};
        fbufCreateInfo.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
        fbufCreateInfo.pNext = NULL;
        fbufCreateInfo.renderPass = deferredFb.renderPass;
        fbufCreateInfo.pAttachments = attachments.data();
        fbufCreateInfo.attachmentCount = static_cast<uint32_t>(attachments.size());
        fbufCreateInfo.width = deferredFb.width;
        fbufCreateInfo.height = deferredFb.height;
        fbufCreateInfo.layers = 1;

        VK_CHECK_RESULT(vkCreateFramebuffer(device, &fbufCreateInfo, nullptr, &deferredFb.frameBuffer));
        // Create sampler to sample from the color attachments
        VkSamplerCreateInfo sampler = vkTools::initializers::samplerCreateInfo();
        sampler.magFilter = VK_FILTER_LINEAR;
        sampler.minFilter = VK_FILTER_LINEAR;
        sampler.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
        sampler.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        sampler.addressModeV = sampler.addressModeU;
        sampler.addressModeW = sampler.addressModeU;
        sampler.mipLodBias = 0.0f;
        sampler.maxAnisotropy = 0;
        sampler.minLod = 0.0f;
        sampler.maxLod = 1.0f;
        sampler.borderColor = VK_BORDER_COLOR_FLOAT_OPAQUE_WHITE;
        VK_CHECK_RESULT(vkCreateSampler(device, &sampler, nullptr, &deferredFb.sampler));
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
