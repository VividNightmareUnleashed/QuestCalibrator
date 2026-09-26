#include "stdafx.h"
#include "UiLayout.h"

FlexLayout::FlexLayout() : origin(0.0f, 0.0f)
{
	config = YGConfigNew();
	// Browser semantics for anything left unsaid, so a reference's CSS maps
	// one to one. Sizes round to whole pixels, as the draw list wants.
	YGConfigSetUseWebDefaults(config, true);
	YGConfigSetPointScaleFactor(config, 1.0f);
	root = YGNodeNewWithConfig(config);
}

FlexLayout::~FlexLayout()
{
	YGNodeFreeRecursive(root);
	YGConfigFree(config);
}

YGNodeRef FlexLayout::Add(YGNodeRef parent)
{
	YGNodeRef node = YGNodeNewWithConfig(config);
	YGNodeInsertChild(parent, node, YGNodeGetChildCount(parent));
	return node;
}

YGNodeRef FlexLayout::Row(YGNodeRef parent)
{
	YGNodeRef node = Add(parent);
	YGNodeStyleSetFlexDirection(node, YGFlexDirectionRow);
	return node;
}

YGNodeRef FlexLayout::Text(YGNodeRef parent, ImFont *font, const char *text)
{
	YGNodeRef node = Add(parent);
	const ImVec2 size = font->CalcTextSizeA(font->LegacySize, FLT_MAX, 0.0f, text);
	YGNodeStyleSetWidth(node, size.x);
	YGNodeStyleSetHeight(node, font->LegacySize);
	// Text is the one thing that must not give way to a neighbour.
	YGNodeStyleSetFlexShrink(node, 0.0f);
	return node;
}

void FlexLayout::Compute(ImVec2 at, float width, float height)
{
	origin = at;
	YGNodeCalculateLayout(root, width, height, YGDirectionLTR);
}

FlexRect FlexLayout::Rect(YGNodeRef node) const
{
	// Yoga positions are relative to the parent; sum them up to the root.
	float x = 0.0f, y = 0.0f;
	for (YGNodeRef n = node; n; n = YGNodeGetOwner(n))
	{
		x += YGNodeLayoutGetLeft(n);
		y += YGNodeLayoutGetTop(n);
	}
	FlexRect r;
	r.min = ImVec2(origin.x + x, origin.y + y);
	r.max = ImVec2(r.min.x + YGNodeLayoutGetWidth(node), r.min.y + YGNodeLayoutGetHeight(node));
	return r;
}
