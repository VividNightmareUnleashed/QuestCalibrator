#pragma once

#include <imgui/imgui.h>
#include <yoga/Yoga.h>

// Flex layout for the screens, computed by Yoga (the flexbox engine behind
// React Native) and read back as screen rectangles that the ImGui draw list
// and widgets are then placed at. A screen that is designed on the web comes
// with its own flex, gap, padding and size values; those go on the nodes
// verbatim, with the Yoga calls named after the CSS properties, instead of
// being turned into offset arithmetic by hand. Web defaults are on, so a node
// is a row that can shrink, as it would be in a browser.
//
// The tree is built once per frame, laid out, read, and thrown away; Yoga
// lays out a few dozen nodes in microseconds, and a persistent tree would
// have to be kept in step with widgets that ImGui already rebuilds each
// frame. Nothing here draws: a node is a place, and what stands there is the
// caller's business.

struct FlexRect
{
	ImVec2 min, max;
	float W() const { return max.x - min.x; }
	float H() const { return max.y - min.y; }
	ImVec2 Size() const { return ImVec2(W(), H()); }
	ImVec2 Center() const { return ImVec2((min.x + max.x) * 0.5f, (min.y + max.y) * 0.5f); }
};

class FlexLayout
{
public:
	FlexLayout();
	~FlexLayout();
	FlexLayout(const FlexLayout &) = delete;
	FlexLayout &operator=(const FlexLayout &) = delete;

	YGNodeRef Root() const { return root; }

	// A new child appended to parent. The layout owns every node it hands out.
	YGNodeRef Add(YGNodeRef parent);
	YGNodeRef Row(YGNodeRef parent);
	YGNodeRef Column(YGNodeRef parent);
	// A leaf the size of one line of text in font; the caller draws the text.
	YGNodeRef Text(YGNodeRef parent, ImFont *font, const char *text);

	// Lay the tree out with the root's top-left corner at origin. A root with
	// no size of its own takes exactly width by height; pass YGUndefined for
	// a dimension the content should decide.
	void Compute(ImVec2 origin, float width, float height);

	// Screen rectangle of a node after Compute.
	FlexRect Rect(YGNodeRef node) const;

private:
	YGConfigRef config;
	YGNodeRef root;
	ImVec2 origin;
};
