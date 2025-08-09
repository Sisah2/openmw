#include "inventorytabsoverlay.hpp"

#include <MyGUI_Button.h>
#include <MyGUI_ImageBox.h>
#include <MyGUI_Window.h>

#include "../mwbase/environment.hpp"
#include "../mwbase/inputmanager.hpp"
#include "../mwbase/windowmanager.hpp"

namespace MWGui
{
    InventoryTabsOverlay::InventoryTabsOverlay()
        : WindowBase("openmw_inventory_tabs.layout")
    {
        MyGUI::Button* tab;
        static const char* kTabIds[] = { "TabMap", "TabInventory", "TabSpells", "TabStats" };

        for (const char* id : kTabIds)
        {
            getWidget(tab, id);
            tab->eventMouseButtonClick += MyGUI::newDelegate(this, &InventoryTabsOverlay::onTabClicked);
            mTabs.push_back(tab);
        }

        MyGUI::ImageBox* image;
        getWidget(image, "BtnL2Image");
        image->setImageTexture(
            MWBase::Environment::get().getInputManager()->getControllerAxisIcon(SDL_CONTROLLER_AXIS_TRIGGERLEFT));

        getWidget(image, "BtnR2Image");
        image->setImageTexture(
            MWBase::Environment::get().getInputManager()->getControllerAxisIcon(SDL_CONTROLLER_AXIS_TRIGGERRIGHT));
    }

    int InventoryTabsOverlay::getHeight()
    {
        MyGUI::Window* window = mMainWidget->castType<MyGUI::Window>();
        return window->getHeight();
    }

    void InventoryTabsOverlay::onTabClicked(MyGUI::Widget* sender)
    {
        if (!MWBase::Environment::get().getWindowManager()->getJournalAllowed())
            return;

        for (int i = 0; i < static_cast<int>(mTabs.size()); i++)
        {
            if (mTabs[i] == sender)
            {
                MWBase::Environment::get().getWindowManager()->setActiveControllerWindow(GM_Inventory, i);
                setTab(i);
                break;
            }
        }
    }

    void InventoryTabsOverlay::setTab(int index)
    {
        for (int i = 0; i < static_cast<int>(mTabs.size()); i++)
            mTabs[i]->setStateSelected(i == index);
    }
}
