#pragma once

#include <hardware_interface/internal/hardware_resource_manager.h>

namespace hardware_interface
{

class PCMState
{
	public:
		PCMState(int32_t pcm_id) :
			pcm_id_(pcm_id)
			, enabled_(false)
			, pressure_switch_(false)
			, compressor_current_(0)
			, closed_loop_control_(0)
			, current_too_high_(false)
			, current_too_high_sticky_(false)
			, shorted_(false)
			, shorted_sticky_(false)
			, not_connected_(false)
			, not_connected_sticky_(false)
			, voltage_fault_(false)
			, voltage_sticky_fault_(false)
			, solenoid_blacklist_(0)
		{
		}

		int32_t  getPcmId(void)                const { return pcm_id_; }
		bool     getEnabled(void)              const { return enabled_; }
		bool     getPressureSwitch(void)       const { return pressure_switch_; }
		double   getCompressorCurrent(void)    const { return compressor_current_; }
		bool     getClosedLoopControl(void)    const { return closed_loop_control_; }
		bool     getCurrentTooHigh(void)       const { return current_too_high_; }
		bool     getCurrentTooHighSticky(void) const { return current_too_high_sticky_; }
		bool     getShorted(void)              const { return shorted_; }
		bool     getShortedSticky(void)        const { return shorted_sticky_; }
		bool     getNotConntected(void)        const { return not_connected_; }
		bool     getNotConnecteSticky(void)    const { return not_connected_sticky_; }
		bool     getVoltageFault(void)         const { return voltage_fault_; }
		bool     getVoltageStickFault(void)    const { return voltage_sticky_fault_; }
		uint32_t getSolenoidBlacklist(void)    const { return solenoid_blacklist_; }

		void setPcmId(int32_t pcm_id)                              { pcm_id_ = pcm_id; }
		void getEnabled(bool enabled)                              { enabled_ = enabled; }
		void getPressureSwitch(bool pressure_switch)               { pressure_switch_ = pressure_switch; }
		void getCompressorCurrent(double compressor_current)       { compressor_current_ = compressor_current; }
		void getClosedLoopControl(bool closed_loop_control)        { closed_loop_control_ = closed_loop_control; }
		void getCurrentTooHigh(bool current_too_high)              { current_too_high_ = current_too_high; }
		void getCurrentTooHighSticky(bool current_too_high_sticky) { current_too_high_sticky_ = current_too_high_sticky; }
		void getShorted(bool shorted)                              { shorted_ = shorted; }
		void getShortedSticky(bool shorted_sticky)                 { shorted_sticky_ = shorted_sticky; }
		void getNotConntected(bool not_connected)                  { not_connected_ = not_connected; }
		void getNotConnecteSticky(bool not_connected_sticky)       { not_connected_sticky_ = not_connected_sticky; }
		void getVoltageFault(bool voltage_fault)                   { voltage_fault_ = voltage_fault; }
		void getVoltageStickFault(bool voltage_sticky_fault)       { voltage_sticky_fault_ = voltage_sticky_fault; }
		void getSolenoidBlacklist(uint32_t solenoid_blacklist)     { solenoid_blacklist_ = solenoid_blacklist; }

	private:
		int32_t pcm_id_;
		bool enabled_;
		bool pressure_switch_;
		double compressor_current_;
		bool closed_loop_control_;
		bool current_too_high_;
		bool current_too_high_sticky_;
		bool shorted_;
		bool shorted_sticky_;
		bool not_connected_;
		bool not_connected_sticky_;
		bool voltage_fault_;
		bool voltage_sticky_fault_;
		uint32_t solenoid_blacklist_;
};

// Match up a name with a pointer to PCMState data
// Used by the controller_manager to give access to controllers
// who want to access PCMState
class PCMStateHandle
{
	public:
		PCMStateHandle(void) : state_(0) {}

		PCMStateHandle(const std::string &name, const PCMState *state) :
			name_(name),
			state_(state)
		{
			if (!state)
				throw HardwareInterfaceException("Cannot create PCM state handle '" + name + "'. State pointer is null.");
		}
		std::string getName(void) const {return name_;}

		const PCMState *operator->() const
		{
			assert(state_);
			return state_;
		}

	private:
		std::string		name_;
		const PCMState *state_;
};

class PCMWritableStateHandle
{
	public:
		PCMWritableStateHandle(void) : state_(0) {}

		PCMWritableStateHandle(const std::string &name, PCMState *state) :
			name_(name),
			state_(state)
		{
			if (!state)
				throw HardwareInterfaceException("Cannot create writable PCM state handle '" + name + "'. State pointer is null.");
		}
		std::string getName(void) const {return name_;}

		PCMState *operator->()
		{
			assert(state_);
			return state_;
		}

	private:
		std::string	 name_;
		PCMState    *state_;
};


class PCMStateInterface: public HardwareResourceManager<PCMStateHandle> {};
} // namespace
