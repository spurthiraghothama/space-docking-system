#include "ui/VdeWindow.hpp"

#include "ui/DockLayout.hpp"
#include "ui/VdeValuesPanel.hpp"
#include "ui/Widgets.hpp"
#include "ui/WindowNames.hpp"
#include "vde/AppState.hpp"
#include "visualization/VdeScene.hpp"

#include <imgui.h>

namespace vde::ui {

VdeWindow::VdeWindow(AppState& app)
	: m_app(app), m_scene(std::make_unique<render::VdeScene>()) {}

ToolbarAction VdeWindow::draw(const std::function<void()>& replayBar) {
	ToolbarAction action = ToolbarAction::None;
	m_graphs.sample(m_app.parameters(), m_app.vdeSnapshot());
	if (m_focusFrames > 0) { ImGui::SetNextWindowFocus(); --m_focusFrames; }
	const bool visible = ImGui::Begin(names::kVdeWindow);
	m_focused = visible && ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows | ImGuiFocusedFlags_DockHierarchy);
	if (visible) {
		ToolbarOptions opt;
		opt.view         = ViewId::Vde;
		opt.recordTag    = "vde_view";
		opt.exportPrefix = "run";
		opt.showAnalysis = opt.showReplay = opt.showConfig = true;
		opt.replayMode   = m_app.replayOnVde();
		opt.timeS        = m_app.vdeSnapshot().timeS;
		action = drawToolbar(m_app, opt);
	}
	DockLayout::submitInner(DockLayout::vdeDockspaceId(), visible);
	ImGui::End();

	drawPanel();
	drawView(replayBar);
	drawGraphs();
	return action;
}

void VdeWindow::drawPanel() {
	const VdeState& s = m_app.vdeSnapshot().vde;
	const VdeLimits& limits = m_app.config().vdeLimits;
	if (ImGui::Begin(names::kVdePanel)) {
		if (m_panelPopped) poppedOutPlaceholder(m_panelPopped);
		else drawVdeValuesPanel(s, limits, m_panelPopped);
	}
	ImGui::End();
	poppedOutWindow("Control Panel – VDE values###VdePanelPopped", m_panelPopped, ImVec2(420, 860),
					[&] { drawVdeValuesPanel(s, limits, m_panelPopped); });
}

void VdeWindow::drawView(const std::function<void()>& replayBar) {
	const SimSnapshot& snap = m_app.vdeSnapshot();
	IRecorder* rec = &m_app.recorder(ViewId::Vde);
	auto body = [&] { m_scene.draw(snap, m_viewPopped, rec, replayBar); };

	ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
	const ImGuiWindowFlags flags = ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse;
	if (ImGui::Begin(names::kVdeView, nullptr, flags)) {
		if (m_viewPopped) poppedOutPlaceholder(m_viewPopped);
		else body();
	}
	ImGui::End();
	poppedOutWindow("VDE View###VdeViewPopped", m_viewPopped, ImVec2(1100, 800), body, flags);
	ImGui::PopStyleVar();
}

void VdeWindow::drawGraphs() {
	if (ImGui::Begin(names::kVdeGraphs)) m_graphs.draw(m_app.parameters());
	ImGui::End();
	m_graphs.drawPopOuts();
}

} // namespace vde::ui
