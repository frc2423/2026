import { useState } from "react";



export function SomeComponent(props: any) {

    const [burritoCount, setBurritoCount] = useState(0);

    function addBurrito() {
        setBurritoCount(burritoCount + 1);
    }

    return (
        <div>
            Hello {props.name}! You have {burritoCount} burritos!
            <button onClick={addBurrito}>Add Burrito</button>
            {[1,2,3].map(number => {
                return <p>Number is: {number}</p>
            })}

        </div>
    );
}