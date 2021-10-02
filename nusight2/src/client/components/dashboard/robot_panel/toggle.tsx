import React from 'react'
import style from './style.css'

function LocalisationToggle() {
  function toggleBallStd(std: number) {
    // Set the score to be the opposite of what it currently is
  }

  return (
    <div>
      <div>
        <span className={style.label}>Ball positions</span>
        <div>
          <div>
            <input type="checkbox" id="std1" onClick={e => toggleBallStd(1)} checked />
            <label htmlFor="std1">Std1</label>
          </div>
          <div>
            <input type="checkbox" id="std2" onClick={e => toggleBallStd(2)} />
            <label htmlFor="std2">Std2</label>
          </div>
          <div>
            <input type="checkbox" id="std3" onClick={e => toggleBallStd(3)} />
            <label htmlFor="std3">Std3</label>
          </div>
        </div>
      </div>
      {/* This is the section I wanted to implement for the visual display toggles of the different robots */}
      {/* I also want to be able to track the position that the robot thinks it is */}
      {/* I should be able to turn off each of these features by a toggle on the robot */}
      {/* <div>
        <span className={style.label}>Robot positions</span>
        <div>
          <input type="checkbox" onClick={e => toggleBallZScore(1)} checked />
          <input type="checkbox" onClick={e => toggleBallZScore(2)} />
          <input type="checkbox" onClick={e => toggleBallZScore(3)} />
        </div>
      </div> */}
    </div>
  )
}

export default LocalisationToggle
