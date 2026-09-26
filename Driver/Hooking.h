#pragma once

#include "Logging.h"
#include <MinHook.h>
#include <atomic>

static_assert(ATOMIC_POINTER_LOCK_FREE == 2,
	"hook function pointers must always be lock-free");

template<class FuncType> class Hook
{
public:
	std::atomic<FuncType> originalFunc{ nullptr };
	explicit Hook(const char *name) : name(name) { }

	bool CreateHookInObjectVTable(void *object, int vtableOffset, void *detourFunction)
	{
		if (targetFunc)
			return Enable();
		if (!object || !detourFunction || vtableOffset < 0)
		{
			LOG("Cannot create hook for %s: invalid object/detour/offset", name);
			return false;
		}

		// For virtual objects, VC++ adds a pointer to the vtable as the first member.
		// To access the vtable, we simply dereference the object.
		void **vtable = *((void ***)object);
		if (!vtable)
		{
			LOG("Cannot create hook for %s: object has no vtable", name);
			return false;
		}

		// The vtable itself is an array of pointers to member functions,
		// in the order they were declared in.
		void *target = vtable[vtableOffset];
		if (!target)
		{
			LOG("Cannot create hook for %s: target function is null", name);
			return false;
		}

		LPVOID original = nullptr;
		auto err = MH_CreateHook(target, detourFunction, &original);
		if (err != MH_OK)
		{
			LOG("Failed to create hook for %s, error: %s", name, MH_StatusToString(err));
			return false;
		}
		targetFunc = target;
		originalFunc.store(reinterpret_cast<FuncType>(original), std::memory_order_release);

		if (!Enable())
		{
			auto removeError = MH_RemoveHook(targetFunc);
			if (removeError == MH_OK || removeError == MH_ERROR_NOT_CREATED)
				Reset();
			else
				LOG("Failed to remove unenabled hook for %s, error: %s",
					name, MH_StatusToString(removeError));
			return false;
		}
		return true;
	}

	bool Disable()
	{
		if (!enabled)
			return true;

		auto error = MH_DisableHook(targetFunc);
		if (error != MH_OK && error != MH_ERROR_DISABLED)
		{
			LOG("Failed to disable hook for %s, error: %s", name, MH_StatusToString(error));
			return false;
		}

		enabled = false;
		return true;
	}

	bool Destroy()
	{
		if (!targetFunc)
			return true;

		if (!Disable())
			return false;

		auto error = MH_RemoveHook(targetFunc);
		if (error != MH_OK && error != MH_ERROR_NOT_CREATED)
		{
			LOG("Failed to remove hook for %s, error: %s", name, MH_StatusToString(error));
			return false;
		}

		Reset();
		return true;
	}

private:
	bool Enable()
	{
		if (enabled)
			return true;

		auto error = MH_EnableHook(targetFunc);
		if (error != MH_OK && error != MH_ERROR_ENABLED)
		{
			LOG("Failed to enable hook for %s, error: %s", name, MH_StatusToString(error));
			return false;
		}

		LOG("Enabled hook for %s", name);
		enabled = true;
		return true;
	}

	void Reset()
	{
		originalFunc.store(nullptr, std::memory_order_release);
		enabled = false;
		targetFunc = nullptr;
	}

	const char *const name;
	bool enabled = false;
	void* targetFunc = nullptr;
};
