
// This function receives battery PERCENTAGES and charging indication
function update_battery(ebat, mbat, charging){
    // console.log("Updating battery")
    if(charging){
        console.log("Robot is charging");
        charge_bar = document.getElementsByClassName("battery_bar_charge");
        empty_bar = document.getElementsByClassName("battery_bar_empty");
        charge_bar[0].style.backgroundColor = "blue";
        charge_bar[0].style.width = "60px";
        empty_bar[0].style.width = "0px";
    }
    else{
        document.getElementById("electronic_charge").style.width = (ebat * 60)+"px";
        document.getElementById("electronic_charge").style.backgroundColor = (ebat < 0.25) ? 'red' : ((ebat < 0.5) ? '#FFFF33' : 'limegreen');
        document.getElementById("electronic_empty").style.width = ((1-ebat) * 60)+"px";
    }
}