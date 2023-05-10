#include <pix_robobus_driver/bms_report.hpp>


BmsReport::BmsReport() {}

void BmsReport::update_bytes(uint8_t bytes_data[8])
{
  for(uint i=0;i<8;i++)
  {
    bytes[i] = bytes_data[i];
  }
}

void BmsReport::Parse() {
  battery_leadacid_voltage_ = batteryleadacidvoltage();
  battery_current_ = batterycurrent();
  battery_voltage_ = batteryvoltage();
  battery_soc_ = batterysoc();
}


// config detail: {'bit': 55, 'is_signed_var': False, 'len': 8, 'name': 'battery_leadacid_voltage', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|25.5]', 'physical_unit': 'V', 'precision': 0.1, 'type': 'double'}
double BmsReport::batteryleadacidvoltage() {
  Byte t0(*(bytes + 6));
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'bit': 23, 'is_signed_var': False, 'len': 16, 'name': 'battery_current', 'offset': -3200.0, 'order': 'motorola', 'physical_range': '[-3200|3353.5]', 'physical_unit': 'A', 'precision': 0.1, 'type': 'double'}
double BmsReport::batterycurrent() {
  Byte t0(*(bytes + 2));
  int32_t x = t0.get_byte(0, 8);

  Byte t1(*(bytes + 3));
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.100000 + -3200.000000;
  return ret;
}

// config detail: {'bit': 7, 'is_signed_var': False, 'len': 16, 'name': 'battery_voltage', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|300]', 'physical_unit': 'V', 'precision': 0.01, 'type': 'double'}
double BmsReport::batteryvoltage() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(0, 8);

  Byte t1(*(bytes + 1));
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.010000;
  return ret;
}

// config detail: {'bit': 39, 'is_signed_var': False, 'len': 8, 'name': 'battery_soc', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|100]', 'physical_unit': '%', 'precision': 1.0, 'type': 'int'}
int BmsReport::batterysoc() {
  Byte t0(*(bytes + 4));
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

