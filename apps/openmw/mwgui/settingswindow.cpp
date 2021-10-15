#include "settingswindow.hpp"

#include <MyGUI_ScrollBar.h>
#include <MyGUI_Window.h>
#include <MyGUI_ComboBox.h>
#include <MyGUI_ScrollView.h>
#include <MyGUI_Gui.h>
#include <MyGUI_TabControl.h>

#include <SDL_video.h>

#include <iomanip>
#include <numeric>
#include <array>

#include <components/debug/debuglog.hpp>
#include <components/misc/stringops.hpp>
#include <components/misc/constants.hpp>
#include <components/widgets/sharedstatebutton.hpp>
#include <components/settings/settings.hpp>
#include <components/resource/resourcesystem.hpp>
#include <components/resource/scenemanager.hpp>
#include <components/sceneutil/lightmanager.hpp>

#include "../mwbase/environment.hpp"
#include "../mwbase/world.hpp"
#include "../mwbase/soundmanager.hpp"
#include "../mwbase/inputmanager.hpp"
#include "../mwbase/mechanicsmanager.hpp"
#include "../mwbase/windowmanager.hpp"

#include "confirmationdialog.hpp"

namespace
{

    std::string textureMipmappingToStr(const std::string& val)
    {
        if (val == "linear")  return "Trilinear";
        if (val == "nearest") return "Bilinear";
        if (val != "none")
            Log(Debug::Warning) << "Warning: Invalid texture mipmap option: "<< val;

        return "Other";
    }

    void parseResolution (int &x, int &y, const std::string& str)
    {
        std::vector<std::string> split;
        Misc::StringUtils::split (str, split, "@(x");
        assert (split.size() >= 2);
        Misc::StringUtils::trim(split[0]);
        Misc::StringUtils::trim(split[1]);
        x = MyGUI::utility::parseInt (split[0]);
        y = MyGUI::utility::parseInt (split[1]);
    }

    bool sortResolutions (std::pair<int, int> left, std::pair<int, int> right)
    {
        if (left.first == right.first)
            return left.second > right.second;
        return left.first > right.first;
    }

    std::string getAspect (int x, int y)
    {
        int gcd = std::gcd (x, y);
        if (gcd == 0)
            return std::string();

        int xaspect = x / gcd;
        int yaspect = y / gcd;
        // special case: 8 : 5 is usually referred to as 16:10
        if (xaspect == 8 && yaspect == 5)
            return "16 : 10";
        return MyGUI::utility::toString(xaspect) + " : " + MyGUI::utility::toString(yaspect);
    }

    const char* checkButtonType = "CheckButton";
    const char* sliderType = "Slider";

    std::string getSettingType(MyGUI::Widget* widget)
    {
        return widget->getUserString("SettingType");
    }

    std::string getSettingName(MyGUI::Widget* widget)
    {
        return widget->getUserString("SettingName");
    }

    std::string getSettingCategory(MyGUI::Widget* widget)
    {
        return widget->getUserString("SettingCategory");
    }

    std::string getSettingValueType(MyGUI::Widget* widget)
    {
        return widget->getUserString("SettingValueType");
    }

    void getSettingMinMax(MyGUI::Widget* widget, float& min, float& max)
    {
        const char* settingMin = "SettingMin";
        const char* settingMax = "SettingMax";
        min = 0.f;
        max = 1.f;
        if (!widget->getUserString(settingMin).empty())
            min = MyGUI::utility::parseFloat(widget->getUserString(settingMin));
        if (!widget->getUserString(settingMax).empty())
            max = MyGUI::utility::parseFloat(widget->getUserString(settingMax));
    }

    void updateMaxLightsComboBox(MyGUI::ComboBox* box)
    {
        constexpr int min = 8;
        constexpr int max = 32;
        constexpr int increment = 8;
        int maxLights = Settings::Manager::getInt("max lights", "Shaders");
        // show increments of 8 in dropdown
        if (maxLights >= min && maxLights <= max && !(maxLights % increment))
            box->setIndexSelected((maxLights / increment)-1);
        else
            box->setIndexSelected(MyGUI::ITEM_NONE);
    }
}

namespace MWGui
{
    void SettingsWindow::configureWidgets(MyGUI::Widget* widget, bool init)
    {
        MyGUI::EnumeratorWidgetPtr widgets = widget->getEnumerator();
        while (widgets.next())
        {
            MyGUI::Widget* current = widgets.current();

            std::string type = getSettingType(current);
            if (type == checkButtonType)
            {
                std::string initialValue = Settings::Manager::getBool(getSettingName(current),
                                                                      getSettingCategory(current))
                        ? "#{sOn}" : "#{sOff}";
                current->castType<MyGUI::Button>()->setCaptionWithReplacing(initialValue);
                if (init)
                    current->eventMouseButtonClick += MyGUI::newDelegate(this, &SettingsWindow::onButtonToggled);
            }
            if (type == sliderType)
            {
                MyGUI::ScrollBar* scroll = current->castType<MyGUI::ScrollBar>();
                std::string valueStr;
                std::string valueType = getSettingValueType(current);
                if (valueType == "Float" || valueType == "Integer" || valueType == "Cell")
                {
                    // TODO: ScrollBar isn't meant for this. should probably use a dedicated FloatSlider widget
                    float min,max;
                    getSettingMinMax(scroll, min, max);
                    float value = Settings::Manager::getFloat(getSettingName(current), getSettingCategory(current));

                    if (valueType == "Cell")
                    {
                        std::stringstream ss;
                        ss << std::fixed << std::setprecision(2) << value/Constants::CellSizeInUnits;
                        valueStr = ss.str();
                    }
                    else if (valueType == "Float")
                    {
                        std::stringstream ss;
                        ss << std::fixed << std::setprecision(2) << value;
                        valueStr = ss.str();
                    }
                    else
                        valueStr = MyGUI::utility::toString(int(value));

                    value = std::max(min, std::min(value, max));
                    value = (value-min)/(max-min);

                    scroll->setScrollPosition(static_cast<size_t>(value * (scroll->getScrollRange() - 1)));
                }
                else
                {
                    int value = Settings::Manager::getInt(getSettingName(current), getSettingCategory(current));
                    valueStr = MyGUI::utility::toString(value);
                    scroll->setScrollPosition(value);
                }
                if (init)
                    scroll->eventScrollChangePosition += MyGUI::newDelegate(this, &SettingsWindow::onSliderChangePosition);
                if (scroll->getVisible())
                    updateSliderLabel(scroll, valueStr);
            }

            configureWidgets(current, init);
        }
    }

    void SettingsWindow::updateSliderLabel(MyGUI::ScrollBar *scroller, const std::string& value)
    {
        std::string labelWidgetName = scroller->getUserString("SettingLabelWidget");
        if (!labelWidgetName.empty())
        {
            MyGUI::TextBox* textBox;
            getWidget(textBox, labelWidgetName);
            std::string labelCaption = scroller->getUserString("SettingLabelCaption");
            labelCaption = Misc::StringUtils::format(labelCaption, value);
            textBox->setCaptionWithReplacing(labelCaption);
        }
    }

    SettingsWindow::SettingsWindow() :
        WindowBase("openmw_settings_window.layout"),
        mKeyboardMode(true)
    {
        bool terrain = Settings::Manager::getBool("distant terrain", "Terrain");
        const std::string widgetName = terrain ? "RenderingDistanceSlider" : "LargeRenderingDistanceSlider";
        MyGUI::Widget* unusedSlider;
        getWidget(unusedSlider, widgetName);
        unusedSlider->setVisible(false);

        configureWidgets(mMainWidget, true);

        setTitle("#{sOptions}");

        getWidget(mSettingsTab, "SettingsTab");
        getWidget(mOkButton, "OkButton");
        getWidget(mResolutionList, "ResolutionList");
        getWidget(mFullscreenButton, "FullscreenButton");
        getWidget(mWindowBorderButton, "WindowBorderButton");
        getWidget(mTextureFilteringButton, "TextureFilteringButton");
        getWidget(mAnisotropyBox, "AnisotropyBox");
        getWidget(mControlsBox, "ControlsBox");
        getWidget(mResetControlsButton, "ResetControlsButton");
        getWidget(mKeyboardSwitch, "KeyboardButton");
        getWidget(mControllerSwitch, "ControllerButton");
        getWidget(mWaterTextureSize, "WaterTextureSize");
        getWidget(mWaterReflectionDetail, "WaterReflectionDetail");
        getWidget(mTonemaperSwitch, "TonemaperSwitch");
        getWidget(mLightingMethodButton, "LightingMethodButton");
        getWidget(mLightsResetButton, "LightsResetButton");
        getWidget(mMaxLights, "MaxLights");

#ifndef WIN32
        // hide gamma controls since it currently does not work under Linux
        MyGUI::ScrollBar *gammaSlider;
        getWidget(gammaSlider, "GammaSlider");
        gammaSlider->setVisible(false);
        MyGUI::TextBox *textBox;
        getWidget(textBox, "GammaText");
        textBox->setVisible(false);
        getWidget(textBox, "GammaTextDark");
        textBox->setVisible(false);
        getWidget(textBox, "GammaTextLight");
        textBox->setVisible(false);
#endif

        if (!Settings::Manager::getBool("enabled", "Groundcover"))
        {
            MyGUI::TextBox *grassDistanceLabel;
            getWidget(grassDistanceLabel, "GrassDistanceLabel");
            grassDistanceLabel->setVisible(false);
            MyGUI::TextBox *grassDensityLabel;
            getWidget(grassDensityLabel, "GrassDensityLabel");
            grassDensityLabel->setVisible(false);

            MyGUI::ScrollBar *grassDistanceSlider;
            getWidget(grassDistanceSlider, "GrassDistanceSlider");
            grassDistanceSlider->setVisible(false);
            MyGUI::ScrollBar *grassDensitySlider;
            getWidget(grassDensitySlider, "GrassDensitySlider");
            grassDensitySlider->setVisible(false);
        }

            MyGUI::Button *PPLButton;
            getWidget(PPLButton, "PPLLightingButton");
            mLightingMethodButton->eventMouseButtonClick += MyGUI::newDelegate(this, &SettingsWindow::onPPLButtonClicked);

	const char *shaderPreset = getenv("OPENMW_SHADERS");
        if(!shaderPreset || strcmp(shaderPreset, "experimental") != 0/* || !MWBase::Environment::get().getResourceSystem()->getSceneManager()->getForceShaders()*/)
	{
            MyGUI::ScrollBar *gammaSlider;
            MyGUI::TextBox *textBox;
            MyGUI::Button *button;

            getWidget(gammaSlider, "ShaderGammaSlider");
            gammaSlider->setVisible(false);
            getWidget(textBox, "ShaderGammaText");
            textBox->setVisible(false);
            getWidget(textBox, "ShaderGammaTextDark");
            textBox->setVisible(false);
            getWidget(textBox, "ShaderGammaTextLight");
            textBox->setVisible(false);

            getWidget(textBox, "RadialFogLabel");
            textBox->setVisible(false);
            getWidget(button, "RadialFogButton");
            button->setVisible(false);

            getWidget(textBox, "ParallaxShadowsLabel");
            textBox->setVisible(false);
            getWidget(button, "ParallaxShadowsButton");
            button->setVisible(false);

            getWidget(textBox, "ClampLightingLabel");
            textBox->setVisible(false);
            getWidget(button, "ClampLightingButton");
            button->setVisible(false);

            getWidget(textBox, "PPLLightingLabel");
            textBox->setVisible(false);
            PPLButton->setVisible(false);

            getWidget(textBox, "UnderwaterFogLabel");
            textBox->setVisible(false);
            getWidget(button, "UnderwaterFogButton");
            button->setVisible(false);

            getWidget(textBox, "TonemaperLabel");
            textBox->setVisible(false);
            mTonemaperSwitch->setVisible(false);
	}

        mMainWidget->castType<MyGUI::Window>()->eventWindowChangeCoord += MyGUI::newDelegate(this, &SettingsWindow::onWindowResize);

        mSettingsTab->eventTabChangeSelect += MyGUI::newDelegate(this, &SettingsWindow::onTabChanged);
        mOkButton->eventMouseButtonClick += MyGUI::newDelegate(this, &SettingsWindow::onOkButtonClicked);
        mTextureFilteringButton->eventComboChangePosition += MyGUI::newDelegate(this, &SettingsWindow::onTextureFilteringChanged);
        mResolutionList->eventListChangePosition += MyGUI::newDelegate(this, &SettingsWindow::onResolutionSelected);

        mWaterTextureSize->eventComboChangePosition += MyGUI::newDelegate(this, &SettingsWindow::onWaterTextureSizeChanged);
        mWaterReflectionDetail->eventComboChangePosition += MyGUI::newDelegate(this, &SettingsWindow::onWaterReflectionDetailChanged);

        mTonemaperSwitch->eventComboChangePosition += MyGUI::newDelegate(this, &SettingsWindow::onTonemaperSwitchChanged);

        mLightingMethodButton->eventComboChangePosition += MyGUI::newDelegate(this, &SettingsWindow::onLightingMethodButtonChanged);
        mLightsResetButton->eventMouseButtonClick += MyGUI::newDelegate(this, &SettingsWindow::onLightsResetButtonClicked);
        mMaxLights->eventComboChangePosition += MyGUI::newDelegate(this, &SettingsWindow::onMaxLightsChanged);

        mKeyboardSwitch->eventMouseButtonClick += MyGUI::newDelegate(this, &SettingsWindow::onKeyboardSwitchClicked);
        mControllerSwitch->eventMouseButtonClick += MyGUI::newDelegate(this, &SettingsWindow::onControllerSwitchClicked);

        center();

        mResetControlsButton->eventMouseButtonClick += MyGUI::newDelegate(this, &SettingsWindow::onResetDefaultBindings);

        // fill resolution list
        int screen = Settings::Manager::getInt("screen", "Video");
        int numDisplayModes = SDL_GetNumDisplayModes(screen);
        std::vector < std::pair<int, int> > resolutions;
        for (int i = 0; i < numDisplayModes; i++)
        {
            SDL_DisplayMode mode;
            SDL_GetDisplayMode(screen, i, &mode);
            resolutions.emplace_back(mode.w, mode.h);
        }
        std::sort(resolutions.begin(), resolutions.end(), sortResolutions);
        for (std::pair<int, int>& resolution : resolutions)
        {
            std::string str = MyGUI::utility::toString(resolution.first) + " x " + MyGUI::utility::toString(resolution.second);
            std::string aspect = getAspect(resolution.first, resolution.second);
            if (!aspect.empty())
                 str = str + " (" + aspect + ")";

            if (mResolutionList->findItemIndexWith(str) == MyGUI::ITEM_NONE)
                mResolutionList->addItem(str);
        }
        highlightCurrentResolution();

        std::string tmip = Settings::Manager::getString("texture mipmap", "General");
        mTextureFilteringButton->setCaption(textureMipmappingToStr(tmip));

        int waterTextureSize = Settings::Manager::getInt("rtt size", "Water");
        if (waterTextureSize >= 512)
            mWaterTextureSize->setIndexSelected(0);
        if (waterTextureSize >= 1024)
            mWaterTextureSize->setIndexSelected(1);
        if (waterTextureSize >= 2048)
            mWaterTextureSize->setIndexSelected(2);

        int waterReflectionDetail = Settings::Manager::getInt("reflection detail", "Water");
        waterReflectionDetail = std::min(5, std::max(0, waterReflectionDetail));
        mWaterReflectionDetail->setIndexSelected(waterReflectionDetail);

        int tonemaperSwitch = Settings::Manager::getInt("tonemaper", "Shaders");
        tonemaperSwitch = std::min(9, std::max(0, tonemaperSwitch));
        mTonemaperSwitch->setIndexSelected(tonemaperSwitch);

        updateMaxLightsComboBox(mMaxLights);

        mWindowBorderButton->setEnabled(!Settings::Manager::getBool("fullscreen", "Video"));

        mKeyboardSwitch->setStateSelected(true);
        mControllerSwitch->setStateSelected(false);
    }

    void SettingsWindow::onTabChanged(MyGUI::TabControl* /*_sender*/, size_t /*index*/)
    {
        resetScrollbars();
    }

    void SettingsWindow::onOkButtonClicked(MyGUI::Widget* _sender)
    {
        MWBase::Environment::get().getWindowManager()->removeGuiMode(GM_Settings);
    }

    void SettingsWindow::onPPLButtonClicked(MyGUI::Widget* _sender)
    {
        std::string message = "This change requires a restart to take effect.";
        MWBase::Environment::get().getWindowManager()->interactiveMessageBox(message, {"#{sOK}"}, true);
    }

    void SettingsWindow::onResolutionSelected(MyGUI::ListBox* _sender, size_t index)
    {
        if (index == MyGUI::ITEM_NONE)
            return;

        ConfirmationDialog* dialog = MWBase::Environment::get().getWindowManager()->getConfirmationDialog();
        dialog->askForConfirmation("#{sNotifyMessage67}");
        dialog->eventOkClicked.clear();
        dialog->eventOkClicked += MyGUI::newDelegate(this, &SettingsWindow::onResolutionAccept);
        dialog->eventCancelClicked.clear();
        dialog->eventCancelClicked += MyGUI::newDelegate(this, &SettingsWindow::onResolutionCancel);
    }

    void SettingsWindow::onResolutionAccept()
    {
        std::string resStr = mResolutionList->getItemNameAt(mResolutionList->getIndexSelected());
        int resX, resY;
        parseResolution (resX, resY, resStr);

        Settings::Manager::setInt("resolution x", "Video", resX);
        Settings::Manager::setInt("resolution y", "Video", resY);

        apply();
    }

    void SettingsWindow::onResolutionCancel()
    {
        highlightCurrentResolution();
    }

    void SettingsWindow::highlightCurrentResolution()
    {
        mResolutionList->setIndexSelected(MyGUI::ITEM_NONE);

        int currentX = Settings::Manager::getInt("resolution x", "Video");
        int currentY = Settings::Manager::getInt("resolution y", "Video");

        for (size_t i=0; i<mResolutionList->getItemCount(); ++i)
        {
            int resX, resY;
            parseResolution (resX, resY, mResolutionList->getItemNameAt(i));

            if (resX == currentX && resY == currentY)
            {
                mResolutionList->setIndexSelected(i);
                break;
            }
        }
    }

    void SettingsWindow::onWaterTextureSizeChanged(MyGUI::ComboBox* _sender, size_t pos)
    {
        int size = 0;
        if (pos == 0)
            size = 512;
        else if (pos == 1)
            size = 1024;
        else if (pos == 2)
            size = 2048;
        Settings::Manager::setInt("rtt size", "Water", size);
        apply();
    }

    void SettingsWindow::onWaterReflectionDetailChanged(MyGUI::ComboBox* _sender, size_t pos)
    {
        unsigned int level = std::min((unsigned int)5, (unsigned int)pos);
        Settings::Manager::setInt("reflection detail", "Water", level);
        apply();
    }

    void SettingsWindow::onTonemaperSwitchChanged(MyGUI::ComboBox* _sender, size_t pos)
    {
        unsigned int level = std::min((unsigned int)9, (unsigned int)pos);
        Settings::Manager::setInt("tonemaper", "Shaders", level);
        apply();
    }

    void SettingsWindow::onLightingMethodButtonChanged(MyGUI::ComboBox* _sender, size_t pos)
    {
        if (pos == MyGUI::ITEM_NONE)
            return;

        std::string message = "This change requires a restart to take effect.";
        MWBase::Environment::get().getWindowManager()->interactiveMessageBox(message, {"#{sOK}"}, true);

        Settings::Manager::setString("lighting method", "Shaders", _sender->getItemNameAt(pos));

        const std::string settingspath = (g_cfgMgr->getUserConfigPath() / "settings.cfg").string();
        Settings::Manager::saveUser(settingspath);

        apply();
    }

    void SettingsWindow::onMaxLightsChanged(MyGUI::ComboBox* _sender, size_t pos)
    {
        int count = 8 * (pos + 1);
        Settings::Manager::setInt("max lights", "Shaders", count);

        std::string message = "This change requires a restart to take effect.";
        MWBase::Environment::get().getWindowManager()->interactiveMessageBox(message, {"#{sOK}"}, true);
        const std::string settingspath = (g_cfgMgr->getUserConfigPath() / "settings.cfg").string();
        Settings::Manager::saveUser(settingspath);

        apply();
        configureWidgets(mMainWidget, false);
    }

    void SettingsWindow::onLightsResetButtonClicked(MyGUI::Widget* _sender)
    {
        std::vector<std::string> buttons = {"#{sYes}", "#{sNo}"};
        std::string message = "Resets to default values, would you like to continue? Changes to lighting method will require a restart.";
        MWBase::Environment::get().getWindowManager()->interactiveMessageBox(message, buttons, true);
        int selectedButton = MWBase::Environment::get().getWindowManager()->readPressedButton();
        if (selectedButton == 1 || selectedButton == -1)
            return;

        constexpr std::array<const char*, 6> settings = {
            "light bounds multiplier",
            "maximum light distance",
            "light fade start",
            "minimum interior brightness",
            "max lights",
            "lighting method",
        };
        for (const auto& setting : settings)
            Settings::Manager::setString(setting, "Shaders", Settings::Manager::mDefaultSettings[{"Shaders", setting}]);

        mLightingMethodButton->setIndexSelected(mLightingMethodButton->findItemIndexWith(Settings::Manager::mDefaultSettings[{"Shaders", "lighting method"}]));
        updateMaxLightsComboBox(mMaxLights);

        apply();
        configureWidgets(mMainWidget, false);
    }

    void SettingsWindow::onButtonToggled(MyGUI::Widget* _sender)
    {
        std::string on = MWBase::Environment::get().getWindowManager()->getGameSettingString("sOn", "On");
        std::string off = MWBase::Environment::get().getWindowManager()->getGameSettingString("sOff", "On");
        bool newState;
        if (_sender->castType<MyGUI::Button>()->getCaption() == on)
        {
            _sender->castType<MyGUI::Button>()->setCaption(off);
            newState = false;
        }
        else
        {
            _sender->castType<MyGUI::Button>()->setCaption(on);
            newState = true;
        }

        if (_sender == mFullscreenButton)
        {
            // check if this resolution is supported in fullscreen
            if (mResolutionList->getIndexSelected() != MyGUI::ITEM_NONE)
            {
                std::string resStr = mResolutionList->getItemNameAt(mResolutionList->getIndexSelected());
                int resX, resY;
                parseResolution (resX, resY, resStr);
                Settings::Manager::setInt("resolution x", "Video", resX);
                Settings::Manager::setInt("resolution y", "Video", resY);
            }

            bool supported = false;
            int fallbackX = 0, fallbackY = 0;
            for (unsigned int i=0; i<mResolutionList->getItemCount(); ++i)
            {
                std::string resStr = mResolutionList->getItemNameAt(i);
                int resX, resY;
                parseResolution (resX, resY, resStr);

                if (i == 0)
                {
                    fallbackX = resX;
                    fallbackY = resY;
                }

                if (resX == Settings::Manager::getInt("resolution x", "Video")
                    && resY  == Settings::Manager::getInt("resolution y", "Video"))
                    supported = true;
            }

            if (!supported && mResolutionList->getItemCount())
            {
                if (fallbackX != 0 && fallbackY != 0)
                {
                    Settings::Manager::setInt("resolution x", "Video", fallbackX);
                    Settings::Manager::setInt("resolution y", "Video", fallbackY);
                }
            }

            mWindowBorderButton->setEnabled(!newState);
        }

        if (getSettingType(_sender) == checkButtonType)
        {
            Settings::Manager::setBool(getSettingName(_sender), getSettingCategory(_sender), newState);
            apply();
            return;
        }
    }

    void SettingsWindow::onTextureFilteringChanged(MyGUI::ComboBox* _sender, size_t pos)
    {
        if(pos == 0)
            Settings::Manager::setString("texture mipmap", "General", "nearest");
        else if(pos == 1)
            Settings::Manager::setString("texture mipmap", "General", "linear");
        else
            Log(Debug::Warning) << "Unexpected option pos " << pos;
        apply();
    }

    void SettingsWindow::onSliderChangePosition(MyGUI::ScrollBar* scroller, size_t pos)
    {
        if (getSettingType(scroller) == "Slider")
        {
            std::string valueStr;
            std::string valueType = getSettingValueType(scroller);
            if (valueType == "Float" || valueType == "Integer" || valueType == "Cell")
            {
                float value = pos / float(scroller->getScrollRange()-1);

                float min,max;
                getSettingMinMax(scroller, min, max);
                value = min + (max-min) * value;
                if (valueType == "Float")
                    Settings::Manager::setFloat(getSettingName(scroller), getSettingCategory(scroller), value);
                else
                    Settings::Manager::setInt(getSettingName(scroller), getSettingCategory(scroller), (int)value);

                if (valueType == "Cell")
                {
                    std::stringstream ss;
                    ss << std::fixed << std::setprecision(2) << value/Constants::CellSizeInUnits;
                    valueStr = ss.str();
                }
                else if (valueType == "Float")
                {
                    std::stringstream ss;
                    ss << std::fixed << std::setprecision(2) << value;
                    valueStr = ss.str();
                }
                else
                    valueStr = MyGUI::utility::toString(int(value));
            }
            else
            {
                Settings::Manager::setInt(getSettingName(scroller), getSettingCategory(scroller), pos);
                valueStr = MyGUI::utility::toString(pos);
            }
            updateSliderLabel(scroller, valueStr);

            apply();
        }
    }

    void SettingsWindow::apply()
    {
        const Settings::CategorySettingVector changed = Settings::Manager::getPendingChanges();
        MWBase::Environment::get().getWorld()->processChangedSettings(changed);
        MWBase::Environment::get().getSoundManager()->processChangedSettings(changed);
        MWBase::Environment::get().getWindowManager()->processChangedSettings(changed);
        MWBase::Environment::get().getInputManager()->processChangedSettings(changed);
        MWBase::Environment::get().getMechanicsManager()->processChangedSettings(changed);
        Settings::Manager::resetPendingChanges();
    }

    void SettingsWindow::onKeyboardSwitchClicked(MyGUI::Widget* _sender)
    {
        if(mKeyboardMode)
            return;
        mKeyboardMode = true;
        mKeyboardSwitch->setStateSelected(true);
        mControllerSwitch->setStateSelected(false);
        updateControlsBox();
        resetScrollbars();
    }

    void SettingsWindow::onControllerSwitchClicked(MyGUI::Widget* _sender)
    {
        if(!mKeyboardMode)
            return;
        mKeyboardMode = false;
        mKeyboardSwitch->setStateSelected(false);
        mControllerSwitch->setStateSelected(true);
        updateControlsBox();
        resetScrollbars();
    }

    void SettingsWindow::updateControlsBox()
    {
        while (mControlsBox->getChildCount())
            MyGUI::Gui::getInstance().destroyWidget(mControlsBox->getChildAt(0));

        MWBase::Environment::get().getWindowManager()->removeStaticMessageBox();
        std::vector<int> actions;
        if(mKeyboardMode)
            actions = MWBase::Environment::get().getInputManager()->getActionKeySorting();
        else
            actions = MWBase::Environment::get().getInputManager()->getActionControllerSorting();

        for (const int& action : actions)
        {
            std::string desc = MWBase::Environment::get().getInputManager()->getActionDescription (action);
            if (desc == "")
                continue;

            std::string binding;
            if(mKeyboardMode)
                binding = MWBase::Environment::get().getInputManager()->getActionKeyBindingName(action);
            else
                binding = MWBase::Environment::get().getInputManager()->getActionControllerBindingName(action);

            Gui::SharedStateButton* leftText = mControlsBox->createWidget<Gui::SharedStateButton>("SandTextButton", MyGUI::IntCoord(), MyGUI::Align::Default);
            leftText->setCaptionWithReplacing(desc);

            Gui::SharedStateButton* rightText = mControlsBox->createWidget<Gui::SharedStateButton>("SandTextButton", MyGUI::IntCoord(), MyGUI::Align::Default);
            rightText->setCaptionWithReplacing(binding);
            rightText->setTextAlign (MyGUI::Align::Right);
            rightText->setUserData(action); // save the action id for callbacks
            rightText->eventMouseButtonClick += MyGUI::newDelegate(this, &SettingsWindow::onRebindAction);
            rightText->eventMouseWheel += MyGUI::newDelegate(this, &SettingsWindow::onInputTabMouseWheel);

            Gui::ButtonGroup group;
            group.push_back(leftText);
            group.push_back(rightText);
            Gui::SharedStateButton::createButtonGroup(group);
        }

        layoutControlsBox();
    }

    void SettingsWindow::updateLightSettings()
    {
        auto lightingMethod = MWBase::Environment::get().getResourceSystem()->getSceneManager()->getLightingMethod();
        std::string lightingMethodStr = SceneUtil::LightManager::getLightingMethodString(lightingMethod);

        mLightingMethodButton->removeAllItems();

        std::array<SceneUtil::LightingMethod, 3> methods = {
            SceneUtil::LightingMethod::FFP,
            SceneUtil::LightingMethod::PerObjectUniform,
            SceneUtil::LightingMethod::SingleUBO,
        };

        for (const auto& method : methods)
        {
            if (!MWBase::Environment::get().getResourceSystem()->getSceneManager()->isSupportedLightingMethod(method))
                continue;

            mLightingMethodButton->addItem(SceneUtil::LightManager::getLightingMethodString(method));
        }

        mLightingMethodButton->setIndexSelected(mLightingMethodButton->findItemIndexWith(lightingMethodStr));
    }

    void SettingsWindow::layoutControlsBox()
    {
        const int h = 18;
        const int w = mControlsBox->getWidth() - 28;
        const int noWidgetsInRow = 2;
        const int totalH = mControlsBox->getChildCount() / noWidgetsInRow * h;

        for (size_t i = 0; i < mControlsBox->getChildCount(); i++)
        {
            MyGUI::Widget * widget = mControlsBox->getChildAt(i);
            widget->setCoord(0, i / noWidgetsInRow * h, w, h);
        }

        // Canvas size must be expressed with VScroll disabled, otherwise MyGUI would expand the scroll area when the scrollbar is hidden
        mControlsBox->setVisibleVScroll(false);
        mControlsBox->setCanvasSize (mControlsBox->getWidth(), std::max(totalH, mControlsBox->getHeight()));
        mControlsBox->setVisibleVScroll(true);
    }

    void SettingsWindow::onRebindAction(MyGUI::Widget* _sender)
    {
        int actionId = *_sender->getUserData<int>();

        _sender->castType<MyGUI::Button>()->setCaptionWithReplacing("#{sNone}");

        MWBase::Environment::get().getWindowManager ()->staticMessageBox ("#{sControlsMenu3}");
        MWBase::Environment::get().getWindowManager ()->disallowMouse();

        MWBase::Environment::get().getInputManager ()->enableDetectingBindingMode (actionId, mKeyboardMode);

    }

    void SettingsWindow::onInputTabMouseWheel(MyGUI::Widget* _sender, int _rel)
    {
        if (mControlsBox->getViewOffset().top + _rel*0.3f > 0)
            mControlsBox->setViewOffset(MyGUI::IntPoint(0, 0));
        else
            mControlsBox->setViewOffset(MyGUI::IntPoint(0, static_cast<int>(mControlsBox->getViewOffset().top + _rel*0.3f)));
    }

    void SettingsWindow::onResetDefaultBindings(MyGUI::Widget* _sender)
    {
        ConfirmationDialog* dialog = MWBase::Environment::get().getWindowManager()->getConfirmationDialog();
        dialog->askForConfirmation("#{sNotifyMessage66}");
        dialog->eventOkClicked.clear();
        dialog->eventOkClicked += MyGUI::newDelegate(this, &SettingsWindow::onResetDefaultBindingsAccept);
        dialog->eventCancelClicked.clear();
    }

    void SettingsWindow::onResetDefaultBindingsAccept()
    {
        if(mKeyboardMode)
            MWBase::Environment::get().getInputManager ()->resetToDefaultKeyBindings ();
        else
            MWBase::Environment::get().getInputManager()->resetToDefaultControllerBindings();
        updateControlsBox ();
    }

    void SettingsWindow::onOpen()
    {
        highlightCurrentResolution();
        updateControlsBox();
        updateLightSettings();
        resetScrollbars();
        MWBase::Environment::get().getWindowManager()->setKeyFocusWidget(mOkButton);
    }

    void SettingsWindow::onWindowResize(MyGUI::Window *_sender)
    {
        layoutControlsBox();
    }

    void SettingsWindow::resetScrollbars()
    {
        mResolutionList->setScrollPosition(0);
        mControlsBox->setViewOffset(MyGUI::IntPoint(0, 0));
    }
}
