#pragma once

constexpr size_t power_of_two(size_t exponent) {
	if (exponent == 0) return 1;
	else return 2 * power_of_two(exponent - 1);
}