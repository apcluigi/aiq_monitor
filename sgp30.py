import i2c
import streams

streams.serial()

class Adafruit_SGP30(i2c.I2C):
    """A driver for the SGP30 gas sensor."""
    
    _SGP30_DEFAULT_I2C_ADDR = 0x58
    _SGP30_FEATURESETS = (0x0020, 0x0022)

    _SGP30_CRC8_POLYNOMIAL = 0x31
    _SGP30_CRC8_INIT = 0xFF
    _SGP30_WORD_LEN = 2
    
    def __init__(self,i2cdrv,address=_SGP30_DEFAULT_I2C_ADDR,clk=100000):
        i2c.I2C.__init__(self,i2cdrv,address,clk)
        self.addr = address
        
    def TVOC(self):
        """Total Volatile Organic Compound in parts per billion."""
        return self.iaq_measure()[1]
    
    def baseline_TVOC(self):
        """Total Volatile Organic Compound baseline value"""
        return self.get_iaq_baseline()[1]
        
    def eCO2(self):
        """Carbon Dioxide Equivalent in parts per million"""
        return self.iaq_measure()[0]
        
    def baseline_eCO2(self):
        """Carbon Dioxide Equivalent baseline value"""
        return self.get_iaq_baseline()[0]
        
    def iaq_init(self):
        """Initialize the IAQ algorithm"""
        # name, command, signals, delay
        self._run_profile(["iaq_init", [0x20, 0x03], 0, 10])
        
    def iaq_measure(self):
        """Measure the eCO2 and TVOC"""
        # name, command, signals, delay
        return self._run_profile(["iaq_measure", [0x20, 0x08], 2, 50])
        
    def get_iaq_baseline(self):
        """Retreive the IAQ algorithm baseline for eCO2 and TVOC"""
        # name, command, signals, delay
        return self._run_profile(["iaq_get_baseline", [0x20, 0x15], 2, 10])
        
    def set_iaq_baseline(self, eCO2, TVOC): 
        """Set the previously recorded IAQ algorithm baseline for eCO2 and TVOC"""
        if eCO2 == 0 and TVOC == 0:
            print("Invalid baseline")
            return
        buffer = []
        for value in [TVOC, eCO2]:
            arr = [value >> 8, value & 0xFF]
            arr.append(self._generate_crc(arr))
            buffer += arr
        self._run_profile(["iaq_set_baseline", [0x20, 0x1E] + buffer, 0, 10])
        
    def set_iaq_humidity(self, gramsPM3): 
        """Set the humidity in g/m3 for eCO2 and TVOC compensation algorithm"""
        tmp = int(gramsPM3 * 256)
        buffer = []
        for value in [tmp]:
            arr = [value >> 8, value & 0xFF]
            arr.append(self._generate_crc(arr))
            buffer += arr
        self._run_profile(["iaq_set_humidity", [0x20, 0x61] + buffer, 0, 10])
        
    def _run_profile(self, profile):
        """Run an SGP 'profile' which is a named command set"""
        name, command, signals, delay = profile

        return self._i2c_read_words_from_cmd(command, delay, signals)
        
    def _i2c_read_words_from_cmd(self, command, delay, reply_size):
        """Run an SGP command query, get a reply and CRC results if necessary"""
        
        self.write(command)
        sleep(delay)
        if not reply_size:
            return None
        crc_result = reply_size * (self._SGP30_WORD_LEN + 1)
        data = self.read(crc_result)
        result = []
        for i in range(reply_size):
            word = [data[3 * i], data[3 * i + 1]]
            crc = data[3 * i + 2]
            if self._generate_crc(word) != crc:
                print("CRC Error")
            result.append(word[0] << 8 | word[1])
        return result
        
    def _generate_crc(self, data):
        """8-bit CRC algorithm for checking data"""
        crc = self._SGP30_CRC8_INIT
        # calculates 8-Bit checksum with given polynomial
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ self._SGP30_CRC8_POLYNOMIAL
                else:
                    crc <<= 1
        return crc & 0xFF