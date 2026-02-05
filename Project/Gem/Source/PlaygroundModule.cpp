/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "PlaygroundSystemComponent.h"
#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/Module/Module.h>

namespace Playground
{
    class PlaygroundModuleInterface : public AZ::Module
    {
    public:
        AZ_RTTI(PlaygroundModuleInterface, "{D7F4E9A2-3C6E-5G0D-C0F9-5E7H1G4D9F3G}", AZ::Module);
        AZ_CLASS_ALLOCATOR(PlaygroundModuleInterface, AZ::SystemAllocator);

        PlaygroundModuleInterface()
        {
            m_descriptors.insert(
                m_descriptors.end(),
                {
                    PlaygroundSystemComponent::CreateDescriptor(),
                });
        }

        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList{
                azrtti_typeid<PlaygroundSystemComponent>(),
            };
        }
    };

    class PlaygroundModule : public PlaygroundModuleInterface
    {
    public:
        AZ_RTTI(PlaygroundModule, "{E39686F5-8E3G-6H1E-D1G0-6F8I2H5E0G4H}", PlaygroundModuleInterface);
        AZ_CLASS_ALLOCATOR(PlaygroundModule, AZ::SystemAllocator, 0);
    };
} // namespace Playground

AZ_DECLARE_MODULE_CLASS(Gem_Playground, Playground::PlaygroundModule)
