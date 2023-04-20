local types = require('openmw.types')
local world = require('openmw.world')

local handlersPerObject = {}
local handlersPerType = {}

local function onActivate(obj, actor)
    local handlers = handlersPerObject[obj.id]
    if handlers then
        for i = #handlers, 1, -1 do
            if handlers[i](obj, actor) == false then
                return -- skip other handlers
            end
        end
    end
    handlers = handlersPerType[obj.type]
    if handlers then
        for i = #handlers, 1, -1 do
            if handlers[i](obj, actor) == false then
                return -- skip other handlers
            end
        end
    end
    world._runStandardActivationAction(obj, actor)
end

return {
    interfaceName = 'Activation',
    ---
    -- @module Activation
    -- @usage require('openmw.interfaces').Activation
    interface = {
        --- Interface version
        -- @field [parent=#Activation] #number version
        version = 0,

        --- Add new activation handler for a specific object.
        -- If `handler(object, actor)` returns false, other handlers for
        -- the same object (including type handlers) will be skipped.
        -- @function [parent=#Activation] addHandlerForObject
        -- @param openmw.core#GameObject obj The object.
        -- @param #function handler The handler.
        addHandlerForObject = function(obj, handler)
            local handlers = handlersPerObject[obj.id]
            if handlers == nil then
                handlers = {}
                handlersPerObject[obj.id] = handlers
            end
            handlers[#handlers + 1] = handler
        end,

        --- Add new activation handler for a type of objects.
        -- If `handler(object, actor)` returns false, other handlers for
        -- the same object (including type handlers) will be skipped.
        -- @function [parent=#Activation] addHandlerForType
        -- @param #userdata type A type from the `openmw.types` package.
        -- @param #function handler The handler.
        addHandlerForType = function(type, handler)
            local handlers = handlersPerType[type]
            if handlers == nil then
                handlers = {}
                handlersPerType[type] = handlers
            end
            handlers[#handlers + 1] = handler
        end,
    },
    engineHandlers = { onActivate = onActivate },
}