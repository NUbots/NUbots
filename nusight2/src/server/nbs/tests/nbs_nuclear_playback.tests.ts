import { FakeClock } from '../../../shared/time/fake_clock'
import { FakeNUClearNetClient } from '../../nuclearnet/fake_nuclearnet_client'
import { FakeNUClearNetServer } from '../../nuclearnet/fake_nuclearnet_server'
import { hashType } from '../../nuclearnet/fake_nuclearnet_server'
import { NbsFrame } from '../nbs_frame_codecs'
import { NbsNUClearPlayback } from '../nbs_nuclear_playback'

describe('NbsNUClearPlayback', () => {
  it('sends packets at the rate specified by the frame timestamps', () => {
    const nuclearnetServer = new FakeNUClearNetServer()
    const nuclearnetClient = new FakeNUClearNetClient(nuclearnetServer)
    const clock = FakeClock.of()
    const playback = new NbsNUClearPlayback(nuclearnetClient, clock)

    const frame1: NbsFrame = {
      timestampInMicroseconds: 0,
      hash: hashType('frame1'),
      payload: Buffer.alloc(8),
    }

    const frame2: NbsFrame = {
      timestampInMicroseconds: 10e6, // 10 seconds
      hash: hashType('frame2'),
      payload: Buffer.alloc(8),
    }

    const frame3: NbsFrame = {
      timestampInMicroseconds: 20e6, // 20 seconds
      hash: hashType('frame3'),
      payload: Buffer.alloc(8),
    }

    nuclearnetClient.connect({ name: 'Bob' })

    jest.spyOn(nuclearnetClient, 'send')

    clock.tick(1000) // Start clock at an offset to test that packets are sent at a relative time.

    playback.write(frame1)
    clock.runAllTimers()
    expect(nuclearnetClient.send).toHaveBeenCalledWith(
      expect.objectContaining({ type: hashType('frame1') }),
    )

    playback.write(frame2)
    clock.runTimersToTime(1009) // Should not have sent packet yet at only 9 elapsed seconds.
    expect(nuclearnetClient.send).not.toHaveBeenCalledWith(
      expect.objectContaining({ type: hashType('frame2') }),
    )
    clock.runTimersToTime(1010) // At 10 elapsed seconds the packet should have been sent.
    expect(nuclearnetClient.send).toHaveBeenCalledWith(
      expect.objectContaining({ type: hashType('frame2') }),
    )

    playback.write(frame3)
    clock.runTimersToTime(1019) // Should not have sent packet yet at only 19 elapsed seconds.
    expect(nuclearnetClient.send).not.toHaveBeenCalledWith(
      expect.objectContaining({ type: hashType('frame3') }),
    )
    clock.runTimersToTime(1020) // At 20 elapsed seconds the packet should have been sent.
    expect(nuclearnetClient.send).toHaveBeenCalledWith(
      expect.objectContaining({ type: hashType('frame3') }),
    )
  })
})
