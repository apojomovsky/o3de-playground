-- Spawns 9 cylinders in 3x3 grid (ROS logo pattern)
local ObstacleSpawner = {
    Properties = {
        CylinderPrefab = { default = SpawnableScriptAssetRef(), description = "Cylinder prefab to spawn" },
        GridSpacing = { default = 1.1, description = "Distance between cylinder centers" },
        CylinderHeight = { default = 0.25, description = "Z offset for cylinder center" }
    }
}

function ObstacleSpawner:OnActivate()
    self.spawnableMediator = SpawnableScriptMediator()
    self.ticket = self.spawnableMediator:CreateSpawnTicket(self.Properties.CylinderPrefab)

    -- Spawn 9 cylinders in 3x3 grid
    local spacing = self.Properties.GridSpacing
    for row = -1, 1 do
        for col = -1, 1 do
            local x = col * spacing
            local y = row * spacing
            local z = self.Properties.CylinderHeight
            local position = Vector3(x, y, z)
            local rotation = Vector3(0, 0, 0)
            self.spawnableMediator:SpawnAndParentAndTransform(
                self.ticket, self.entityId, position, rotation, 1.0)
        end
    end

    Debug.Log("ObstacleSpawner: Spawned 9 cylinders in ROS logo pattern")
end

function ObstacleSpawner:OnDeactivate()
    -- Cleanup if needed
end

return ObstacleSpawner
