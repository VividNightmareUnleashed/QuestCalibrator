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

	// `object` is a non-null interface pointer; callers check it. A hook is
	// created once and only ever disabled after that, so a later call for the
	// same function enables it again, as a second Init in vrserver does.
	bool CreateHookInObjectVTable(void *object, int vtableOffset, void *detourFunction)
	{
		// MSVC places the vtable pointer first in a polymorphic object.
		void **vtable = *((void ***)object);
		if (!vtable)
		{
			LOG("Cannot create hook for %s: object has no vtable", name);
			return false;
		}

		void *target = vtable[vtableOffset];
		if (!target)
		{
			LOG("Cannot create hook for %s: target function is null", name);
			return false;
		}

		if (targetFunc)
		{
			if (target != targetFunc)
			{
				LOG("Cannot hook %s on another implementation: the hook already patches one", name);
				return false;
			}
			return Enable();
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
