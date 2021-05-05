$(document).ready(function () {
    "use strict";
    $('.week_button a').click(function (e) {
        e.preventDefault();
        $('.week_button').removeClass('active');
        parent = $(this).parent()
        parent.removeClass('active');
        populate_week_work(parent)
        parent.addClass('active');
    });
});



function populate_week_work(parent) {
    title = parent.find(".title").html()
    if (title == undefined) title = "Week not found"
    information = parent.find(".information").html()
    if (information == undefined) information = "Work of this week hasn't been added yet"
    $('#show_week_work .title').html(title);
    $('#show_week_work .information').html(information);
}

$(document).ready(function () {
    parent = $('.week_button.active')
    populate_week_work(parent)
});