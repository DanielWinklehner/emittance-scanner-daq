import time

def fast_read(ser, timeout=1.0):
    # Our own 'readline()' function
    response = b''

    start_time = time.time()
    while (time.time() - start_time) < timeout:
        resp = ser.read(1)
        if resp:
            response += bytes(resp)
            if resp in [b'\n', b'\r']:
                break
            elif resp == b';':
                # Handle MFC Readout (read in two more bytes for checksum and break)
                response += bytes(ser.read(2))
                break
            elif resp == b'?':
                # stepper motor will sometimes send a '?' without a newline
                # so this clause prevents this function from getting stuck
                break
    #else: # thanks, python
    #    response += b''
    return response
