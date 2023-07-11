#include <pix_sweeping_driver/bms_report.hpp>


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
  wireless_charging_sta_ = wirelesschargingsta();
  battery_soc_ = batterysoc();
  battery_current_ = batterycurrent();
  battery_voltage_ = batteryvoltage();
}


// config detail: {'bit': 55, 'description': '铅酸电池电压', 'is_signed_var': True, 'len': 8, 'name': 'battery_leadacid_voltage', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|25.4]', 'physical_unit': 'V', 'precision': 0.1, 'type': 'double'}
double BmsReport::batteryleadacidvoltage() {
  Byte t0(*(bytes + 6));
  int32_t x = t0.get_byte(0, 8);

  x <<= 24;
  x >>= 24;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'bit': 41, 'description': '无线充电状态', 'is_signed_var': False, 'len': 2, 'name': 'wireless_charging_sta', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int BmsReport::wirelesschargingsta() {
  Byte t0(*(bytes + 5));
  int32_t x = t0.get_byte(0, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 39, 'description': '电池SOC', 'is_signed_var': True, 'len': 8, 'name': 'battery_soc', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|100]', 'physical_unit': '%', 'precision': 1.0, 'type': 'int'}
int BmsReport::batterysoc() {
  Byte t0(*(bytes + 4));
  int32_t x = t0.get_byte(0, 8);

  x <<= 24;
  x >>= 24;

  int ret = x;
  return ret;
}

// config detail: {'bit': 23, 'description': '电池电流', 'is_signed_var': True, 'len': 16, 'name': 'battery_current', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-3200|3353.5]', 'physical_unit': 'A', 'precision': 0.1, 'type': 'double'}
double BmsReport::batterycurrent() {
  Byte t0(*(bytes + 2));
  int32_t x = t0.get_byte(0, 8);

  Byte t1(*(bytes + 3));
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'bit': 7, 'description': '电池电压', 'is_signed_var': True, 'len': 16, 'name': 'battery_voltage', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|300]', 'physical_unit': 'V', 'precision': 0.01, 'type': 'double'}
double BmsReport::batteryvoltage() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(0, 8);

  Byte t1(*(bytes + 1));
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.010000;
  return ret;
}

