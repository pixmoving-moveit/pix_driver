#include <pix_robobus_driver/vcu_pad_transfer.hpp>

VCUPadTransfer::VCUPadTransfer() {}

void VCUPadTransfer::update_bytes(uint8_t bytes_data[8])
{
  for(uint i=0;i<8;i++)
  {
    bytes[i] = bytes_data[i];
  }
}

void VCUPadTransfer::Parse() {
  v_c_u__pad_auto_start_ = vcupadautostart();
}


// config detail: {'bit': 4, 'is_signed_var': False, 'len': 1, 'name': 'v_c_u__pad_auto_start', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool VCUPadTransfer::vcupadautostart() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(4, 1);

  bool ret = x;
  return ret;
}