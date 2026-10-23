#pragma once
#include "ui/AnalysisWindow.hpp"
#include "ui/ConfigXmlDialog.hpp"
#include "ui/DockLayout.hpp"
#include "ui/LogConsole.hpp"
#include "ui/ReplayDialog.hpp"
#include "ui/SpacecraftWindow.hpp"
#include "ui/VdeWindow.hpp"

#include <string>

namespace vde { class AppState; }

namespace vde::ui {

// Top-level UI: menu bar, dockspaces, all windows and dialogs.
class MainUi {
public:
	explicit MainUi(AppState& app);
	void frame();
	// Title for the OS window, follows the focused VDE / Spacecraft window.
	const std::string& osWindowTitle() const { return m_osTitle; }

private:
	void drawMenuBar();
	void handle(ToolbarAction action);

	AppState&        m_app;
	DockLayout       m_layout;
	VdeWindow        m_vde;
	SpacecraftWindow m_spacecraft;
	AnalysisWindow   m_analysis;
	LogConsole       m_log;
	ConfigXmlDialog  m_configDialog;
	ReplayDialog     m_replayDialog;
	std::string      m_osTitle = "VDE Docking Simulator – VDE";
	bool             m_showDemo = false;
};

} // namespace vde::ui
